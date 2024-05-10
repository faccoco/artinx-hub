#include <chrono>
#include "Utility.hpp"
#include <cstdlib>
#include <exception>
#ifdef ARTINX_HIK
#include "BlackBoard.hpp"
#include "CameraBase.hpp"
#include "CameraFrame.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "Timer.hpp"
#include "Utility.hpp"
#include <cstdlib>
#include <exception>

#include "SuppressWarningBegin.hpp"

#include <CameraParams.h>
#include <MvCameraControl.h>
#include <MvErrorDefine.h>
#include <caf/event_based_actor.hpp>
#include <fmt/core.h>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

#include "SuppressWarningEnd.hpp"
#include <algorithm>
#include <atomic>
#include <cctype>
#include <csignal>
#include <cstring>
#include <optional>
#include <string_view>
#include <thread>
#include <type_traits>

namespace {
    template <typename Fun, typename... Args>
    std::optional<std::result_of_t<std::decay_t<Fun>(std::decay_t<Args>...)>> timeoutTask(Fun&& fun, Duration dur, Args... args) {
        typename std::result_of<typename std::decay<Fun>::type(typename std::decay<Args>::type...)>::type* result = nullptr;
        std::thread taskThread{ [&]() {
            std::signal(SIGABRT, [](int) -> void {});
            try {
                *result = fun(std::forward<Args>(args)...);
            } catch(std::exception& e) {
            }
        } };
        taskThread.join();
        std::this_thread::sleep_for(dur);
        if(result == nullptr) {
            taskThread.~thread();
        }
        return std::move(*result);
    }

    void showDriverVersion() {
        auto version = MV_CC_GetSDKVersion();
        logInfo(fmt::format("Hik driver version: {}.{}.{}.{}", (version & 0xFF000000) >> 24, (version & 0xFF0000) >> 16,
                            (version & 0xFF00) >> 8, version & 0xFF));
    }

}  // namespace

#define CheckErrorCode(ERROR_CODE)                                                            \
    {                                                                                         \
        if((ERROR_CODE) != MV_OK) {                                                           \
            logError(fmt::format("Hik Driver error: {:x} at line {}", ERROR_CODE, __LINE__)); \
        }                                                                                     \
    }

    template <typename Fun, typename... Args>
    void checkErrorCodeTimeout(Fun&& fn, Duration dur, std::string_view timeoutMsg, Args... args) {
        if(auto res = timeoutTask(std::forward<Fun>(fn), dur, std::forward<Args>(args)...)) {
            checkErrorCode(res.value());
        } else {
            logError(timeoutMsg);
        }
    }


class HikDriver final : public CameraBase {
private:
    Identifier mKey;
    MV_CC_DEVICE_INFO_LIST mDeviceList;
    MV_CC_DEVICE_INFO* mDeviceInfo;
    MV_IMAGE_BASIC_INFO mImageInfo;
    void* mCameraHandle;
    TimePoint mLastSend = SynchronizedClock::instance().now();
    std::atomic<bool> mStartFlag{ false };

    void restartCamera() noexcept override {
        CheckErrorCode(MV_CC_CloseDevice(mCameraHandle));
        CheckErrorCode(MV_CC_DestroyHandle(mCameraHandle));
        openCamera(mConfig.cameraConnectMaxTry, std::chrono::milliseconds(mConfig.cameraConnectTimeout));
    };

    static void newFrame(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) {
        auto instance = static_cast<HikDriver*>(pUser);
        auto timeStamp = SynchronizedClock::instance().now();

        if(std::chrono::duration_cast<std::chrono::nanoseconds>(timeStamp - instance->mLastSend).count() / 1000000.0 <
               1e3 / instance->mConfig.fps ||
           !instance->mStartFlag.load(std::memory_order_consume)) {
            return;
        }
        instance->mLastSend = timeStamp;

        cv::Mat bgr{ pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3, static_cast<void*>(pData) };
        instance->sendFrame(bgr);
    }

    MV_CC_DEVICE_INFO* getDeviceInfo() {
        for(uint32_t i = 0; i < mDeviceList.nDeviceNum; ++i) {
            if(0 ==
               std::strncmp(reinterpret_cast<char*>(mDeviceList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber),
                            mConfig.identifier.c_str(), mConfig.identifier.length())) {
                return mDeviceList.pDeviceInfo[i];
            }
        }
        logError(fmt::format("Can find camera with serial number {}", mConfig.identifier));
        return nullptr;
    }

    void sendFrame(cv::Mat& frame) {
        if(mConfig.flip) {
            cv::Mat flipped;
            cv::flip(frame, flipped, -1);
            std::swap(frame, flipped);
        }
        reportFrameRate(SynchronizedClock::instance().now());

        Pose gunPose{};
        if(mHeadKey.has_value()) {
            gunPose = BlackBoard::instance().get<HeadInfo>(mHeadKey.value())->pose;
        } else {
            gunPose.yaw = glm::half_pi<double>();
        }

        CameraFrame frameData;
        frameData.lastUpdate = SynchronizedClock::instance().now();
        frameData.info.cameraMatrix = mCameraMatrix;
        frameData.info.distCoefficients = mDistCoefficients;
        frameData.info.identifier = mCameraSerialNumber;
        frameData.info.width = frame.cols;
        frameData.info.height = frame.rows;
        frameData.info.tfRobot2Camera = clcTfRobot2Camera(gunPose);
        frame.copyTo(frameData.frame);

        HubLogger::visualLog("Hik Camera: Camera send an image");
        sendAll(image_frame_atom_v,
                BlackBoard::instance().updateSync(mKey, std::move(frameData), static_cast<std::string_view>(mConfig.cameraName)));
        mSendFlag.store(true, std::memory_order_release);
    }

    void openCamera(uint32_t retryTimes = 1, Duration retryInterval = 0ms) {
        for(uint32_t i = 0;; ++i) {
            CheckErrorCode(MV_CC_EnumDevices(MV_USB_DEVICE, &mDeviceList));
            if(mDeviceList.nDeviceNum > 0) {
                break;
            } else if(i >= retryTimes) {
                HubLogger::visualLog(fmt::format(R"(No hik camera found after {} time(s) try)", retryTimes));
                logError(fmt::format(R"(No hik camera found after {} time(s) try)", retryTimes));
                return;
            }
            logInfo(fmt::format("{}th time try to find hik camera, found 0 device", i));
            std::this_thread::sleep_for(retryInterval);
        }
        logInfo(fmt::format("{} camera(s) found", mDeviceList.nDeviceNum));

        std::transform(mConfig.identifier.begin(), mConfig.identifier.end(), mConfig.identifier.begin(),
                       [](const char c) { return std::toupper(c); });
        mDeviceInfo = mConfig.openMode == "Index" ? mDeviceList.pDeviceInfo[0] : getDeviceInfo();
        mCameraSerialNumber = std::string(reinterpret_cast<char*>(mDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber));
        CheckErrorCode(MV_CC_CreateHandleWithoutLog(&mCameraHandle, mDeviceInfo));
        CheckErrorCode(MV_CC_CloseDevice(mCameraHandle));
        CheckErrorCode(MV_CC_OpenDevice(mCameraHandle, MV_ACCESS_Control));
        CheckErrorCode(MV_CC_GetImageInfo(mCameraHandle, &mImageInfo));
        logInfo(fmt::format("Resolution for {}: {} x {}", mCameraSerialNumber, mImageInfo.nWidthMax, mImageInfo.nHeightMax));
        loadCalibration(mCameraSerialNumber, static_cast<uint32_t>(mImageInfo.nWidthMax),
                        static_cast<uint32_t>(mImageInfo.nHeightMax), mConfig.fov, mCameraMatrix, mDistCoefficients);
        CheckErrorCode(MV_CC_SetEnumValue(mCameraHandle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF));
        CheckErrorCode(MV_CC_SetEnumValue(mCameraHandle, "ExposureMode", MV_EXPOSURE_MODE_TIMED));
        CheckErrorCode(MV_CC_SetFloatValue(mCameraHandle, "ExposureTime", mConfig.exposureTime * 1.0e6));
        CheckErrorCode(MV_CC_SetFloatValue(mCameraHandle, "Gain", mConfig.gain));
        CheckErrorCode(MV_CC_SetEnumValue(mCameraHandle, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS));
        CheckErrorCode(
            MV_CC_SetEnumValue(mCameraHandle, "BalanceWhiteAuto",
                               mConfig.enableAutoWhiteBalance ? MV_BALANCEWHITE_AUTO_CONTINUOUS : MV_BALANCEWHITE_AUTO_OFF));
        CheckErrorCode(MV_CC_SetBayerCvtQuality(mCameraHandle, 1));
        CheckErrorCode(MV_CC_RegisterImageCallBackForBGR(mCameraHandle, newFrame, this));
        CheckErrorCode(MV_CC_StartGrabbing(mCameraHandle));
    }

    void closeCamera() {
        CheckErrorCode(MV_CC_StopGrabbing(mCameraHandle));
        CheckErrorCode(MV_CC_CloseDevice(mCameraHandle));
        CheckErrorCode(MV_CC_DestroyHandle(mCameraHandle));
    }

public:
    HikDriver(caf::actor_config& base, const HubConfig& config, std::string name)
        : CameraBase{ base, config, std::move(name) }, mKey{ generateKey(this) } {
        showDriverVersion();
        openCamera();
    }

    ~HikDriver() override {
        closeCamera();
    }

    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                    mStartFlag.store(true, std::memory_order_release);
                },
                 [this](update_head_atom, GroupMask, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(update_head_atom, GroupMask, TypedIdentifier<HeadInfo>);
                     mHeadKey = key;
                 } };
    }
};
HUB_REGISTER_CLASS(HikDriver);
#undef CheckErrorCode
#endif
