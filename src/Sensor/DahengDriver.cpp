#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <glm/gtc/matrix_transform.hpp>
#include <opencv2/opencv.hpp>
#pragma warning(push, 0)
#include <GxIAPI.h>
#pragma warning(pop)

struct DahengDriverSettings final {
    std::string openMode;
    std::string identifier;
    double fps;
    double fov;
    double exposureTime;
    uint32_t decimation;
};

enum class OpenMode { Index, SerialNumber };

template <class Inspector>
bool inspect(Inspector& f, DahengDriverSettings& x) {
    return f.object(x).fields(
        f.field("openMode", x.openMode).invariant([](const std::string& v) { return v == "Index" || v == "SerialNumber"; }),
        f.field("identifier", x.identifier),
        f.field("fps", x.fps).fallback(30.0).invariant([](double v) { return v >= 1.0 && v <= 500.0; }), f.field("fov", x.fov),
        f.field("exposureTime", x.exposureTime), f.field("decimation", x.decimation));
}

static void checkGXStatus(const GX_STATUS status) {
    if(status != GX_STATUS_SUCCESS) {
        const auto error = "GX Error: " + std::to_string(status);
        logError(error.c_str());
    }
}

class DahengLibGuard final : Unmovable {
public:
    DahengLibGuard() {
        checkGXStatus(GXInitLib());
    }
    ~DahengLibGuard() {
        checkGXStatus(GXCloseLib());
    }
};

static void initLib() {
    static DahengLibGuard guard;
}

class DahengDriver final : public HubHelper<caf::event_based_actor, DahengDriverSettings, image_frame_atom> {
    Identifier mKey;
    GX_DEV_HANDLE mDevice;
    bool mStartFlag = false;

    static constexpr auto pixelFormat = GX_PIXEL_FORMAT_BAYER_RG8;
    static constexpr auto pixelCast = cv::COLOR_BayerRG2RGB_EA;
    static constexpr auto pixelStorageFormat = CV_8UC1;

    void newFrameImpl(Clock::time_point timeStamp, const cv::Mat& frame, uint32_t width, uint32_t height) {
        cv::Mat bgr;
        cv::cvtColor(frame, bgr, pixelCast);

        CameraFrame frameData;
        frameData.lastUpdate = timeStamp;
        frameData.info.fov = mConfig.fov;
        frameData.info.width = width;
        frameData.info.height = height;
        // TODO: transform
        frameData.info.transform = Transform<FrameOfReference::Gun, FrameOfReference::Camera, true>(glm::identity<glm::dmat4>());
        frameData.frame = std::move(bgr);

        BlackBoard::instance().updateSync(mKey, std::move(frameData));
        sendAll(image_frame_atom_v, mKey);
    }

#ifdef ARTINX_DAHENG_USB2
    std::thread mCaptureThread;
    bool mRunning = true;

    void acquireOneFrame() {}
#else
    void newFrame(GX_FRAME_CALLBACK_PARAM* pFrameData) {
        if(pFrameData->status != GX_FRAME_STATUS_SUCCESS || !mStartFlag)
            return;

        // std::cout << "Frame " << (static_cast<double>(Clock::now().time_since_epoch().count()) / Clock::period::den) << " " <<
        // pFrameData->nWidth << " x "
        //          << pFrameData->nHeight << std::endl;

        const auto timeStamp = SynchronizedClock::instance().now();  // TODO: propagation time and internal timer

        // TODO: reduce reallocation
        cv::Mat frame{ cv::Size{ pFrameData->nWidth, pFrameData->nHeight }, pixelStorageFormat };
        memcpy(frame.data, pFrameData->pImgBuf, pFrameData->nImgSize);
        newFrameImpl(timeStamp, frame, pFrameData->nWidth, pFrameData->nHeight);
    }
#endif

public:
    DahengDriver(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(DahengDriver).hash_code() } {
        initLib();

        GX_OPEN_PARAM deviceDesc;
        deviceDesc.accessMode = GX_ACCESS_CONTROL;
        deviceDesc.openMode = mConfig.openMode == "Index" ? GX_OPEN_MODE::GX_OPEN_INDEX : GX_OPEN_MODE::GX_OPEN_SN;
        deviceDesc.pszContent = mConfig.identifier.data();

        checkGXStatus(GXOpenDevice(&deviceDesc, &mDevice));

#ifdef ARTINX_DAHENG_USB2
        checkGXStatus(GXSetEnum(mDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS));
#else
        checkGXStatus(GXSetEnum(mDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON));
        checkGXStatus(GXSetFloat(mDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, mConfig.fps));
#endif

        // checkGXStatus(GXSetEnum(device, GX_ENUM_FLAT_FIELD_CORRECTION, GX_ENUM_FLAT_FIELD_CORRECTION_ON));
        // checkGXStatus(GXSetEnum(device, GX_ENUM_NOISE_REDUCTION_MODE, GX_NOISE_REDUCTION_MODE_ON));
        // checkGXStatus(GXSetFloat(device, GX_FLOAT_NOISE_REDUCTION, 2.0));

#ifndef ARTINX_DAHENG_USB2
        checkGXStatus(GXSetEnum(mDevice, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED));
#endif
        checkGXStatus(GXSetEnum(mDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF));
        // checkGXStatus(GXSetEnum(device, GX_ENUM_EXPOSURE_TIME_MODE, GX_EXPOSURE_TIME_MODE_ULTRASHORT));

        GX_FLOAT_RANGE range;
        checkGXStatus(GXGetFloatRange(mDevice, GX_FLOAT_EXPOSURE_TIME, &range));

        checkGXStatus(GXSetFloat(mDevice, GX_FLOAT_EXPOSURE_TIME, 1000000.0 * mConfig.exposureTime));

        checkGXStatus(GXSetEnum(mDevice, GX_ENUM_PIXEL_FORMAT, pixelFormat));

        /*
        if(mConfig.decimation) {
            checkGXStatus(GXSetEnum(mDevice, GX_ENUM_BINNING_HORIZONTAL_MODE, GX_BINNING_VERTICAL_MODE_AVERAGE));
            checkGXStatus(GXSetEnum(mDevice, GX_ENUM_BINNING_VERTICAL_MODE, GX_BINNING_VERTICAL_MODE_AVERAGE));
            checkGXStatus(GXSetInt(mDevice, GX_INT_BINNING_HORIZONTAL, mConfig.decimation));
            checkGXStatus(GXSetInt(mDevice, GX_INT_BINNING_VERTICAL, mConfig.decimation));
            //checkGXStatus(GXSetInt(mDevice, GX_INT_DECIMATION_HORIZONTAL, mConfig.decimation));
            //checkGXStatus(GXSetInt(mDevice, GX_INT_DECIMATION_VERTICAL, mConfig.decimation));
        }
         */

        uint32_t targetWidth = 640, targetHeight = 480;

        int64_t width, height;
        checkGXStatus(GXGetInt(mDevice, GX_INT_WIDTH_MAX, &width));
        checkGXStatus(GXGetInt(mDevice, GX_INT_HEIGHT_MAX, &height));
#ifndef ARTINX_DAHENG_USB2
        checkGXStatus(GXSetInt(mDevice, GX_INT_WIDTH, targetWidth));
        checkGXStatus(GXSetInt(mDevice, GX_INT_HEIGHT, targetHeight));
        checkGXStatus(GXSetInt(mDevice, GX_INT_OFFSET_X, (width - targetWidth) / 2));
        checkGXStatus(GXSetInt(mDevice, GX_INT_OFFSET_Y, (height - targetHeight) / 2));
#endif
        // checkGXStatus(GXSetEnum(mDevice, GX_ENUM_BINNING_HORIZONTAL_MODE, GX_BINNING_VERTICAL_MODE_AVERAGE));

#ifdef ARTINXHUB_WINDOWS
        auto bImplementPacketSize = false;
        checkGXStatus(GXIsImplemented(mDevice, GX_INT_GEV_PACKETSIZE, &bImplementPacketSize));
        if(bImplementPacketSize) {
            uint32_t unPacketSize = 0;
            checkGXStatus(GXGetOptimalPacketSize(mDevice, &unPacketSize));
            checkGXStatus(GXSetInt(mDevice, GX_INT_GEV_PACKETSIZE, unPacketSize));
        }
#endif

#ifndef ARTINX_DAHENG_USB2
        checkGXStatus(GXRegisterCaptureCallback(mDevice, this, [](GX_FRAME_CALLBACK_PARAM* pFrameData) {
            static_cast<DahengDriver*>(pFrameData->pUserParam)->newFrame(pFrameData);
        }));
#endif

        checkGXStatus(GXSendCommand(mDevice, GX_COMMAND_ACQUISITION_START));

#ifdef ARTINX_DAHENG_USB2
        mCaptureThread = std::thread([this] {
            auto current = Clock::now();
            const auto increment = static_cast<Clock::duration>(static_cast<int64_t>(1'000'000'000 / mConfig.fps));

            GX_FRAME_DATA data;

            int64_t payloadSize;
            checkGXStatus(GXGetInt(mDevice, GX_INT_PAYLOAD_SIZE, &payloadSize));
            std::vector<uint8_t> payload(payloadSize);
            data.pImgBuf = payload.data();

            while(mRunning) {
                std::this_thread::sleep_until(current);
                GXGetImage(mDevice, &data, 100);
                if(data.nStatus == GX_FRAME_STATUS_SUCCESS) {
                    const auto timeStamp = SynchronizedClock::instance().now();  // TODO: propagation time and internal timer

                    // TODO: reduce reallocation
                    cv::Mat frame{ cv::Size{ data.nWidth, data.nHeight }, pixelStorageFormat };
                    memcpy(frame.data, data.pImgBuf, data.nImgSize);
                    newFrameImpl(timeStamp, frame, data.nWidth, data.nHeight);
                    current += increment;
                }
            }
        });
#endif
    }
    ~DahengDriver() override {
#ifdef ARTINX_DAHENG_USB2
        mRunning = false;
        mCaptureThread.join();
#endif
        checkGXStatus(GXSendCommand(mDevice, GX_COMMAND_ACQUISITION_STOP));
#ifndef ARTINX_DAHENG_USB2
        checkGXStatus(GXUnregisterCaptureCallback(mDevice));
#endif
        checkGXStatus(GXCloseDevice(mDevice));
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) { mStartFlag = true; } };
    }
};

HUB_REGISTER_CLASS(DahengDriver);
