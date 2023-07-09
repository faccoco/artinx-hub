#include "BlackBoard.hpp"
#include "CameraBase.hpp"
#include "CameraFrame.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "PixelType.h"
#include "Timer.hpp"

#include "SuppressWarningBegin.hpp"

#include <CameraParams.h>
#include <MvCameraControl.h>
#include <MvErrorDefine.h>
#include <algorithm>
#include <caf/event_based_actor.hpp>
#include <cctype>
#include <cstring>
#include <fmt/core.h>
#include <glm/ext/matrix_transform.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>

#include "SuppressWarningEnd.hpp"

namespace {
    void showDriverVersion() {
        auto version = MV_CC_GetSDKVersion();
        logInfo(fmt::format("Hik driver version: {}.{}.{}.{}", (version & 0xFF000000) >> 24, (version & 0xFF0000) >> 16,
                            (version & 0xFF00) >> 8, version & 0xFF));
    }

    void checkErrorCode(const unsigned int errorCode) {
        if(errorCode != MV_OK)
            logError(fmt::format("Hik Driver error: {0:x}", errorCode));
    }

}  // namespace

class HikDriver final : public CameraBase {
private:
    Identifier mKey;
    MV_CC_DEVICE_INFO_LIST mDeviceList;
    MV_CC_DEVICE_INFO* mDeviceInfo;
    MV_IMAGE_BASIC_INFO mImageInfo;
    void* mCameraHandle;
    std::atomic<TimePoint> mLastSend{ SynchronizedClock::instance().now() };

    static void newFrame(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) {
        auto instance = static_cast<HikDriver*>(pUser);
        auto timeStamp = SynchronizedClock::instance().now();

        if(std::chrono::duration_cast<std::chrono::nanoseconds>(timeStamp - instance->mLastSend.load(std::memory_order_acquire))
                   .count() /
               1000000.0 <
           1e3 / instance->mConfig.fps) {
            return;
        } else {
            instance->mLastSend.store(timeStamp, std::memory_order_release);
        }
        cv::Mat bgr{ pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3, static_cast<void*>(pData) };
        CameraFrame frame{
            SynchronizedClock::instance().now(),
            { instance->mTfGun2Camera,
              instance->mHeadKey.has_value() ? BlackBoard::instance().get<HeadInfo>(instance->mHeadKey.value())->tfRobot2Gun :
                                               glm::identity<glm::dmat4>(),
              instance->mCameraSerialNumber, instance->mCameraMatrix, instance->mDistCoefficients, pFrameInfo->nWidth,
              pFrameInfo->nHeight },
        };
        bgr.copyTo(frame.frame);
        instance->sendFrame(std::move(frame));
    }

    MV_CC_DEVICE_INFO* getDeviceInfo() {
        for(uint32_t i = 0; i < mDeviceList.nDeviceNum; ++i)
            if(0 ==
               std::strncmp(reinterpret_cast<char*>(mDeviceList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber),
                            mConfig.identifier.c_str(), mConfig.identifier.length())) {
                return mDeviceList.pDeviceInfo[i];
            }
        logError(fmt::format("Can find camera with serial number {}", mConfig.identifier));
        return nullptr;
    }

    void sendFrame(CameraFrame&& frame) {
        reportFrameRate(SynchronizedClock::instance().now());
        sendAll(image_frame_atom_v,
                BlackBoard::instance().updateSync(mKey, std::move(frame), std::string_view(mConfig.cameraName)));
    }

public:
    HikDriver(caf::actor_config& base, const HubConfig& config) : CameraBase{ base, config }, mKey{ generateKey(this) } {
        showDriverVersion();
        checkErrorCode(MV_CC_EnumDevices(MV_USB_DEVICE, &mDeviceList));
        if(mDeviceList.nDeviceNum == 0) {
            throw std::runtime_error("No camera found");
        } else {
            logInfo(fmt::format("{} camera(s) found", mDeviceList.nDeviceNum));
        }
        std::transform(mConfig.identifier.begin(), mConfig.identifier.end(), mConfig.identifier.begin(),
                       [](const char c) { return std::toupper(c); });
        mDeviceInfo = mConfig.openMode == "Index" ? mDeviceList.pDeviceInfo[0] : getDeviceInfo();
        mCameraSerialNumber = std::string(reinterpret_cast<char*>(mDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber));
        checkErrorCode(MV_CC_CreateHandleWithoutLog(&mCameraHandle, mDeviceInfo));
        checkErrorCode(MV_CC_OpenDevice(mCameraHandle, MV_ACCESS_ControlSwitchEnableWithKey));
        checkErrorCode(MV_CC_GetImageInfo(mCameraHandle, &mImageInfo));
        logInfo(fmt::format("Resolution for {}: {} x {}", mCameraSerialNumber, mImageInfo.nWidthMax, mImageInfo.nHeightMax));
        loadCalibration(mConfig.disableUndistort, mCameraSerialNumber, static_cast<uint32_t>(mImageInfo.nWidthMax),
                        static_cast<uint32_t>(mImageInfo.nHeightMax), mConfig.fov, mCameraMatrix, mDistCoefficients,
                        mDoUndistort);
        checkErrorCode(MV_CC_SetEnumValue(mCameraHandle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF));
        checkErrorCode(MV_CC_SetEnumValue(mCameraHandle, "ExposureMode", MV_EXPOSURE_MODE_TIMED));
        checkErrorCode(MV_CC_SetFloatValue(mCameraHandle, "ExposureTime", mConfig.exposureTime * 1.0e6));
        checkErrorCode(MV_CC_SetFloatValue(mCameraHandle, "Gain", mConfig.gain));
        checkErrorCode(MV_CC_SetEnumValue(mCameraHandle, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS));
        checkErrorCode(
            MV_CC_SetEnumValue(mCameraHandle, "BalanceWhiteAuto",
                               mConfig.enableAutoWhiteBalance ? MV_BALANCEWHITE_AUTO_CONTINUOUS : MV_BALANCEWHITE_AUTO_OFF));
        checkErrorCode(MV_CC_RegisterImageCallBackForBGR(mCameraHandle, newFrame, this));
        checkErrorCode(MV_CC_StartGrabbing(mCameraHandle));
    }

    ~HikDriver() override {
        MV_CC_StopGrabbing(mCameraHandle);
        MV_CC_CloseDevice(mCameraHandle);
        MV_CC_DestroyHandle(mCameraHandle);
    }

    caf::behavior make_behavior() override {
        return { [](start_atom) {} };
    }
};
HUB_REGISTER_CLASS(HikDriver);
