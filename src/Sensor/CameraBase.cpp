#include "CameraBase.hpp"
#include "CameraFrame.hpp"

CameraBase::CameraBase(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}

void CameraBase::loadCalibration(const bool disableUndistort, const std::string& identifier, const uint32_t width,
                                 const uint32_t height, const double fallbackFov, cv::Mat& cameraMatrix,
                                 cv::Mat& distCoefficients, bool& undistort) {
    const auto inputFileName = "./data/camera_calibration/" + identifier + ".xml";

    const cv::FileStorage fs(inputFileName, cv::FileStorage::READ);
    if(!disableUndistort && std::filesystem::exists(inputFileName) && fs.isOpened()) {
        fs["camera_matrix"] >> cameraMatrix;
        fs["distortion_coefficients"] >> distCoefficients;
        undistort = true;
    } else {
        logWarning(fmt::format("Failed to get calibration info for serial number {}. Use default fov {} instead.", identifier,
                               fallbackFov));
        cameraMatrix = (cv::Mat_<double>(3, 3) << width / 2.0 / std::tan(glm::radians(fallbackFov) / 2.0), 0,
                        static_cast<double>(width) / 2.0, 0, height / 2.0 / std::tan(glm::radians(fallbackFov) / 2.0),
                        static_cast<double>(height) / 2.0, 0, 0, 1);
        distCoefficients = cv::Mat{};
        undistort = false;
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
