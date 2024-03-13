#include "BlackBoard.hpp"
#include "CameraBase.hpp"
#include "CameraFrame.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "Timer.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include <GxIAPI.h>
#include <caf/event_based_actor.hpp>
#include <csignal>
#include <exception>
#include <fmt/core.h>
#include <magic_enum.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <tuple>

#include "SuppressWarningEnd.hpp"

static void checkGXStatus(const GX_STATUS status) {
    static const std::string gxStatusList[] = {
        "Success",
        "There is an unspecified internal error that is not expected to occur",
        "The TL library cannot be found",
        "The device is not found",
        "The current device is in an offline status",
        "Invalid parameter. Generally, the pointer is NULL or the input IP and other parameter formats are invalid",
        "Invalid handle",
        "The interface is invalid, which refers to software interface logic error",
        "The function is currently inaccessible or the device access mode is incorrect",
        "The user request buffer is insufficient: the user input buffer size during the read operation is less than the actual "
        "need",
        "The type of FeatureID used by the user is incorrect, such as an integer interface using a floating-point function code",
        "The value written by the user is crossed",
        "This function is not currently supported",
        "There is no call to initialize the interface",
        "Timeout error"
    };
    if(status != GX_STATUS_SUCCESS) {
        if(status >= -14 && status <= 0) {
            logError(fmt::format("GX Error {}: {}", status, gxStatusList[-status]));
        } else {
            logError(fmt::format("GX Error {}: unknown error code", status));
        }
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

class DahengDriver final : public CameraBase {
    GX_DEV_HANDLE mDevice;
    bool mStartFlag = false;
    Identifier mKey;

    static constexpr auto PixelFormat = GX_PIXEL_FORMAT_BAYER_RG8;
    static constexpr auto PixelCast = cv::COLOR_BayerRG2RGB_EA;
    static constexpr auto PixelStorageFormat = CV_8UC1;

    void restartCamera() noexcept override {
        HubLogger::visualLog("Daheng camera down, terminate the whole program");
        logError("Daheng camera down, terminate the whole program");
        terminateSystem(*this, false);
        raise(SIGABRT);
    }

    void newFrameImpl(Clock::time_point timeStamp, const cv::Mat& frame, uint32_t width, uint32_t height) {
        cv::Mat bgr;
        cv::cvtColor(frame, bgr, PixelCast);

        if(mConfig.flip) {
            cv::Mat flipped;
            cv::flip(bgr, flipped, -1);
            std::swap(bgr, flipped);
        }

        reportFrameRate(timeStamp);

        Pose gunPose{};
        if(mHeadKey.has_value()) {
            gunPose = BlackBoard::instance().get<HeadInfo>(mHeadKey.value())->pose;
        } else {
            gunPose.yaw = glm::half_pi<double>();
        }

        CameraFrame frameData;
        frameData.lastUpdate = timeStamp;
        frameData.info.cameraMatrix = mCameraMatrix;
        frameData.info.distCoefficients = mDistCoefficients;
        frameData.info.identifier = mCameraSerialNumber;
        frameData.info.width = width;
        frameData.info.height = height;
        frameData.info.tfRobot2Camera = clcTfRobot2Camera(gunPose);

        frameData.frame = std::move(bgr);

        HubLogger::visualLog("Daheng Camera: Camera send an image");
        sendAll(
            image_frame_atom_v,
            BlackBoard::instance().updateSync(mKey, std::move(frameData), static_cast<std ::string_view>(mConfig.cameraName)));
        mSendFlag.store(true, std::memory_order_release);
    }

#ifdef ARTINX_DAHENG_USB2
    std::thread mCaptureThread;
    std::atomic<bool> mRunning = true;

    void acquireOneFrame() {}
#else

    void newFrame(GX_FRAME_CALLBACK_PARAM* pFrameData) {
        if(pFrameData->status != GX_FRAME_STATUS_SUCCESS || !mStartFlag) {
            return;
        }

        const auto timeStamp = SynchronizedClock::instance().now();  // TODO: propagation time and internal timer

        cv::Mat frame(cv::Size{ pFrameData->nWidth, pFrameData->nHeight }, PixelStorageFormat,
                      const_cast<void*>(pFrameData->pImgBuf));
        //        memcpy(frame.data, pFrameData->pImgBuf, pFrameData->nImgSize);
        newFrameImpl(timeStamp, frame, pFrameData->nWidth, pFrameData->nHeight);
    }

#endif

    void openCamera(size_t tryTime = 1, Duration retryInterval = 1ms) {
        GX_OPEN_PARAM deviceDesc;
        deviceDesc.accessMode = GX_ACCESS_CONTROL;
        deviceDesc.openMode = mConfig.openMode == "Index" ? GX_OPEN_MODE::GX_OPEN_INDEX : GX_OPEN_MODE::GX_OPEN_SN;
        deviceDesc.pszContent = mConfig.identifier.data();
        checkGXStatus(GXOpenDevice(&deviceDesc, &mDevice));
        char strSN[256];
        size_t size = 256;
        checkGXStatus(GXGetString(mDevice, GX_STRING_DEVICE_SERIAL_NUMBER, strSN, &size));
        mCameraSerialNumber = strSN;
        logInfo(fmt::format("open camera {}", mCameraSerialNumber));

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

        checkGXStatus(GXSetEnum(mDevice, GX_ENUM_PIXEL_FORMAT, PixelFormat));

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

        int64_t width, height;
        checkGXStatus(GXGetInt(mDevice, GX_INT_WIDTH_MAX, &width));
        checkGXStatus(GXGetInt(mDevice, GX_INT_HEIGHT_MAX, &height));

        logInfo(fmt::format("Resolution for {}: {} x {}", mCameraSerialNumber, width, height));

        checkGXStatus(GXSetInt(mDevice, GX_INT_WIDTH, width));
        checkGXStatus(GXSetInt(mDevice, GX_INT_HEIGHT, height));
        checkGXStatus(GXSetInt(mDevice, GX_INT_OFFSET_X, 0));
        checkGXStatus(GXSetInt(mDevice, GX_INT_OFFSET_Y, 0));

        if(mConfig.enableAutoWhiteBalance) {
            checkGXStatus(GXSetEnum(mDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS));
        }

        if(abs(mConfig.gain - 0.0) > DBL_EPSILON) {
            GX_FLOAT_RANGE gainRange;
            checkGXStatus(GXGetFloatRange(mDevice, GX_FLOAT_GAIN, &gainRange));
            logInfo(fmt::format("Current camera gain range: {} to {}", gainRange.dMin, gainRange.dMax));
            if(mConfig.gain > gainRange.dMax) {
                mConfig.gain = gainRange.dMax;
            } else if(mConfig.gain < gainRange.dMin) {
                mConfig.gain = gainRange.dMin;
            }
            checkGXStatus(GXSetEnum(mDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL));
            checkGXStatus(GXSetFloat(mDevice, GX_FLOAT_GAIN, mConfig.gain));
        }

        loadCalibration(mCameraSerialNumber, static_cast<uint32_t>(width), static_cast<uint32_t>(height), mConfig.fov,
                        mCameraMatrix, mDistCoefficients);

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

            while(mRunning.load(std::memory_order_consume)) {
                std::this_thread::sleep_until(current);
                GXGetImage(mDevice, &data, 100);
                if(data.nStatus == GX_FRAME_STATUS_SUCCESS && mStartFlag) {
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

    void closeCamera() {
#ifdef ARTINX_DAHENG_USB2
        mRunning.store(false, std::memory_order_release);
        mCaptureThread.join();
#endif
        checkGXStatus(GXSendCommand(mDevice, GX_COMMAND_ACQUISITION_STOP));
#ifndef ARTINX_DAHENG_USB2
        checkGXStatus(GXUnregisterCaptureCallback(mDevice));
#endif
        checkGXStatus(GXCloseDevice(mDevice));
        // mDevice = nullptr;
    }

public:
    DahengDriver(caf::actor_config& base, const HubConfig& config, std::string name)
        : CameraBase{ base, config, std::move(name) }, mKey{ generateKey(this) } {
        initLib();
        openCamera();
    }

    ~DahengDriver() override {
        closeCamera();
    }

    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                    mStartFlag = true;
                },
                 [this](update_head_atom, GroupMask, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(update_head_atom, GroupMask, TypedIdentifier<HeadInfo>);
                     mHeadKey = key;
                 }

        };
    }
};

HUB_REGISTER_CLASS(DahengDriver);