#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <opencv2/opencv.hpp>
#pragma warning(push, 0)
#include <GxIAPI.h>
#pragma warning(pop)

struct DahengDriverSettings final {
    std::string serialNumber;
    double fps;
    uint32_t width;
    uint32_t height;
};

template <class Inspector>
bool inspect(Inspector& f, DahengDriverSettings& x) {
    return f.object(x).fields(f.field("serialNumber", x.serialNumber),
                              f.field("fps", x.fps).fallback(30.0).invariant([](double v) { return v >= 1.0 && v <= 500.0; }),
                              f.field("width", x.width), f.field("height", x.height));
}

static void checkGXStatus(const GX_STATUS status) {
    if(status != GX_STATUS_SUCCESS) {
        const auto error = "GX Error: " + std::to_string(status);
        CAF_RAISE_ERROR(error.c_str());
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
private:
    Identifier mKey;
    GX_DEV_HANDLE mDevice;
    bool mStartFlag;

    static constexpr auto pixelFormat = GX_PIXEL_FORMAT_BAYER_RG8;
    static constexpr auto pixelCast = cv::COLOR_BayerRG2RGB_EA;
    static constexpr auto pixelStorageFormat = CV_8UC1;

    void newFrame(GX_FRAME_CALLBACK_PARAM* pFrameData) {
        if(pFrameData->status != GX_FRAME_STATUS_SUCCESS || !mStartFlag)
            return;

        const auto timeStamp = SynchronizedClock::now();  // TODO: propagation time and internal timer

        cv::Mat frame{ cv::Size{ pFrameData->nWidth, pFrameData->nHeight }, pixelStorageFormat };
        memcpy(frame.data, pFrameData->pImgBuf, pFrameData->nImgSize);

        CameraFrame frameData;
        frameData.lastUpdate = timeStamp;
        cv::Mat bgr;
        cv::cvtColor(frame, bgr, pixelCast);
        cv::resize(bgr,frameData.frame,cv::Size{960,600});
        // TODO: frameData.info;
        frameData.info.width = 960;
        frameData.info.height = 600;
        frameData.info.fov = 49.0;

        BlackBoard::instance().updateSync(mKey, std::move(frameData));
        sendAll(image_frame_atom_v, mKey);
    }

public:
    DahengDriver(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(DahengDriver).hash_code() }, mStartFlag{ false } {
        initLib();

        GX_OPEN_PARAM deviceDesc;
        deviceDesc.accessMode = GX_ACCESS_CONTROL;
        deviceDesc.openMode = GX_OPEN_MODE::GX_OPEN_INDEX;
        deviceDesc.pszContent = "1";

        // checkGXStatus(GXOpenDeviceByIndex(1, &mDevice));
        checkGXStatus(GXOpenDevice(&deviceDesc, &mDevice));

        checkGXStatus(GXSetEnum(mDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON));

        checkGXStatus(GXSetFloat(mDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, mConfig.fps));

        // checkGXStatus(GXSetEnum(device, GX_ENUM_FLAT_FIELD_CORRECTION, GX_ENUM_FLAT_FIELD_CORRECTION_ON));
        // checkGXStatus(GXSetEnum(device, GX_ENUM_NOISE_REDUCTION_MODE, GX_NOISE_REDUCTION_MODE_ON));
        // checkGXStatus(GXSetFloat(device, GX_FLOAT_NOISE_REDUCTION, 2.0));

        checkGXStatus(GXSetEnum(mDevice, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED));
        checkGXStatus(GXSetEnum(mDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF));
        // checkGXStatus(GXSetEnum(device, GX_ENUM_EXPOSURE_TIME_MODE, GX_EXPOSURE_TIME_MODE_ULTRASHORT));

        /*
        GX_FLOAT_RANGE range;
        checkGXStatus(GXGetFloatRange(device, GX_FLOAT_EXPOSURE_TIME, &range));
        */
        checkGXStatus(GXSetFloat(mDevice, GX_FLOAT_EXPOSURE_TIME, 10000.0 ));

        checkGXStatus(GXSetEnum(mDevice, GX_ENUM_PIXEL_FORMAT, pixelFormat));

        checkGXStatus(GXSetInt(mDevice, GX_INT_WIDTH, mConfig.width));
        checkGXStatus(GXSetInt(mDevice, GX_INT_HEIGHT, mConfig.height));
        checkGXStatus(GXSetInt(mDevice, GX_INT_OFFSET_X, 0));
        checkGXStatus(GXSetInt(mDevice, GX_INT_OFFSET_Y, 0));

#ifdef ARTINXHUB_WINDOWS
        auto bImplementPacketSize = false;
        checkGXStatus(GXIsImplemented(mDevice, GX_INT_GEV_PACKETSIZE, &bImplementPacketSize));
        if(bImplementPacketSize) {
            uint32_t unPacketSize = 0;
            checkGXStatus(GXGetOptimalPacketSize(mDevice, &unPacketSize));
            checkGXStatus(GXSetInt(mDevice, GX_INT_GEV_PACKETSIZE, unPacketSize));
        }
#endif

        checkGXStatus(GXRegisterCaptureCallback(mDevice, this, [](GX_FRAME_CALLBACK_PARAM* pFrameData) {
            static_cast<DahengDriver*>(pFrameData->pUserParam)->newFrame(pFrameData);
        }));

        checkGXStatus(GXSendCommand(mDevice, GX_COMMAND_ACQUISITION_START));
    }
    ~DahengDriver() {
        checkGXStatus(GXSendCommand(mDevice, GX_COMMAND_ACQUISITION_STOP));
        checkGXStatus(GXUnregisterCaptureCallback(mDevice));
        checkGXStatus(GXCloseDevice(mDevice));
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) { mStartFlag = true; } };
    }
};

HUB_REGISTER_CLASS(DahengDriver);
