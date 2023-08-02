#pragma once
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>
#include <glm/gtc/matrix_transform.hpp>

struct CameraBaseSettings {
    std::string openMode;
    std::string identifier;
    std::string cameraName;
    double fps;
    double fov;
    double exposureTime;
    bool flip;
    bool enableAutoWhiteBalance;
    double gain;
    bool isAtGun;
    glm::dvec3 offset;  // based on gun
    double yaw;         // in degree
    double pitch;       // in degree
};

template <class Inspector>
bool inspect(Inspector& f, CameraBaseSettings& x) {
    return f.object(x).fields(
        f.field("openMode", x.openMode).invariant([](std::string_view v) { return v == "Index" || v == "SerialNumber"; }),
        f.field("identifier", x.identifier), f.field("cameraName", x.cameraName).fallback("origin"),
        f.field("fps", x.fps).fallback(30.0).invariant([](const double v) { return v >= 1.0 && v <= 500.0; }),
        f.field("fov", x.fov), f.field("exposureTime", x.exposureTime), f.field("flip", x.flip).fallback(false),
        f.field("enableAutoWhiteBalance", x.enableAutoWhiteBalance).fallback(false), f.field("gain", x.gain).fallback(0.0),
        f.field("isAtGun", x.isAtGun).fallback(true), f.field("dx", x.offset.x).fallback(0.0),
        f.field("dy", x.offset.y).fallback(0.0), f.field("dz", x.offset.z).fallback(0.0), f.field("yaw", x.yaw).fallback(0.0),
        f.field("pitch", x.pitch).fallback(0.0));
}

class CameraBase : public HubHelper<caf::event_based_actor, CameraBaseSettings, image_frame_atom, rune_image_frame_atom> {
protected:
    std::string mCameraSerialNumber;

    std::optional<Identifier> mHeadKey;
    double mYaw = glm::radians(mConfig.yaw), mPitch = glm::radians(mConfig.pitch);
    cv::Mat mCameraMatrix;
    cv::Mat mDistCoefficients;
    // first rotate yaw, counterclockwise is positive, second rotate pitch, up is positive
    glm::dmat4 rotateMat =
        glm::rotate(glm::rotate(glm::identity<glm::dmat4>(), -glm::radians<double>(mConfig.pitch), glm::dvec3{ 1, 0, 0 }),
                    -glm::radians<double>(mConfig.yaw), glm::dvec3{ 0, 1, 0 });
    const Transform<FrameOfRef::Gun, FrameOfRef::Camera, true> mTfGun2Camera = glm::translate(rotateMat, -mConfig.offset);
    std::deque<Clock::rep> mLastFrames;

    CameraBase(caf::actor_config& base, const HubConfig& config, std::string name);

    static void loadCalibration(const std::string& identifier, uint32_t width, uint32_t height, double fallbackFov,
                                cv::Mat& cameraMatrix, cv::Mat& distCoefficients);

    void reportFrameRate(Clock::time_point timeStamp);

    Transform<FrameOfRef::Robot, FrameOfRef::Camera, true> clcTfRobot2Camera(const Pose& gunPose);
};
