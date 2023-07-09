#pragma once
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
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
    bool disableUndistort;
    bool enableAutoWhiteBalance;
    double gain;
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
        f.field("disableUndistort", x.disableUndistort).fallback(false),
        f.field("enableAutoWhiteBalance", x.enableAutoWhiteBalance).fallback(false), f.field("gain", x.gain).fallback(0.0),
        f.field("dx", x.offset.x).fallback(0.0), f.field("dy", x.offset.y).fallback(0.0), f.field("dz", x.offset.z).fallback(0.0),
        f.field("yaw", x.yaw).fallback(0.0), f.field("pitch", x.pitch).fallback(0.0));
}

class CameraBase : public HubHelper<caf::event_based_actor, CameraBaseSettings, image_frame_atom> {
protected:
    std::string mCameraSerialNumber;
    bool mDoUndistort{};

    std::optional<Identifier> mHeadKey;
    cv::Mat mCameraMatrix;
    cv::Mat mDistCoefficients;
    glm::dmat4 rotateMat =
        glm::rotate(glm::rotate(glm::identity<glm::dmat4>(), -glm::radians<double>(mConfig.pitch), glm::dvec3{ 1, 0, 0 }),
                    -glm::radians<double>(mConfig.yaw), glm::dvec3{ 0, 1, 0 });
    const Transform<FrameOfRef::Gun, FrameOfRef::Camera, true> mTfGun2Camera = glm::translate(rotateMat, -mConfig.offset);
    std::deque<Clock::rep> mLastFrames;

    CameraBase(caf::actor_config& base, const HubConfig& config);

    static void loadCalibration(bool disableUndistort, const std::string& identifier, uint32_t width, uint32_t height,
                                double fallbackFov, cv::Mat& cameraMatrix, cv::Mat& distCoefficients, bool& undistort);

    void reportFrameRate(Clock::time_point timeStamp);
};
