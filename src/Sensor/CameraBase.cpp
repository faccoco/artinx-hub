#include "Utility.hpp"
#include "CameraBase.hpp"


CameraBase::CameraBase(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}

void CameraBase::loadCalibration(const std::string& identifier, const uint32_t width,
                                 const uint32_t height, const double fallbackFov, cv::Mat& cameraMatrix,
                                 cv::Mat& distCoefficients) {
    const auto inputFileName = "./data/camera_calibration/" + identifier + ".xml";

    const cv::FileStorage fs(inputFileName, cv::FileStorage::READ);
    if(std::filesystem::exists(inputFileName) && fs.isOpened()) {
        fs["camera_matrix"] >> cameraMatrix;
        fs["distortion_coefficients"] >> distCoefficients;
    } else {
        logWarning(fmt::format("Failed to get calibration info for serial number {}. Use default fov {} instead.", identifier,
                               fallbackFov));
        cameraMatrix = (cv::Mat_<double>(3, 3) << width / 2.0 / std::tan(glm::radians(fallbackFov) / 2.0), 0,
                        static_cast<double>(width) / 2.0, 0, height / 2.0 / std::tan(glm::radians(fallbackFov) / 2.0),
                        static_cast<double>(height) / 2.0, 0, 0, 1);
        distCoefficients = cv::Mat{};
    }
}

void CameraBase::reportFrameRate(const Clock::time_point timeStamp) {
    const auto current = timeStamp.time_since_epoch().count();
    mLastFrames.push_back(current);

    while(current - mLastFrames.front() > 1'000'000'000)
        mLastFrames.pop_front();

    const auto delta = std::max(static_cast<Clock::rep>(1), current - mLastFrames.front());
    const auto fps = (static_cast<double>(mLastFrames.size()) - 1.0) * 1e9 / static_cast<double>(delta);
    HubLogger::watch("fps", static_cast<uint32_t>(fps));
}

Transform<FrameOfRef::Robot, FrameOfRef::Camera, true> CameraBase::clcTfRobot2Camera(const Pose& gunPose) {
    double gunRoll = gunPose.roll, gunPitch = gunPose.pitch, gunYaw = gunPose.yaw;
    if (mConfig.isAtGun){
        const Transform<FrameOfRef::Robot, FrameOfRef::Gun, true> tfRobot2Gun = glm::lookAtRH(glm::dvec3{ 0.0, 0.0, 0.0 },
                                                                                        glm::dvec3{ std::cos(gunPitch) * std::cos(gunYaw), mConfig.offset.y + std::sin(gunPitch),
                                                                                                    mConfig.offset.z - std::cos(gunPitch) * std::sin(gunYaw) },
                                                                                        glm::dvec3{ sin(gunRoll), cos(gunRoll), 0.0 });
        return combine(tfRobot2Gun, mTfGun2Camera);
    }else{
        double cameraYaw = normalizeAngle(glm::radians(mConfig.yaw) + gunYaw);
        double cameraPitch = glm::radians(mConfig.pitch);
        double cameraRoll = 0.0;
        return glm::lookAtRH(glm::dvec3{ 0.0, 0.0, 0.0 },
                                                      glm::dvec3{ mConfig.offset.x + std::cos(mConfig.pitch) * std::cos(cameraYaw), mConfig.offset.y + std::sin(cameraPitch),
                                                                  mConfig.offset.z - std::cos(cameraPitch) * std::sin(cameraYaw) },
                                                      glm::dvec3{ sin(cameraRoll), cos(cameraRoll), 0.0 });
    }
}
