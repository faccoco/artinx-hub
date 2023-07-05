#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
// #include "RadarInfo.hpp"

#include "SuppressWarningBegin.hpp"

#include <algorithm>
#include <caf/event_based_actor.hpp>
#include <ctime>
#include <filesystem>
#include <fmt/core.h>
#include <glm/gtc/matrix_transform.hpp>
#include <iomanip>
#include <opencv2/core/persistence.hpp>
#include <opencv2/videoio.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "SuppressWarningEnd.hpp"
#include "Timer.hpp"

namespace fs = std::filesystem;
using namespace std::literals;

std::time_t parseTimePoint(const std::string& filePath) {
    size_t beg = filePath.find_last_of("/") + 1;
    std::string fileName = filePath.substr(beg, filePath.find_last_of('.') - beg);
    std::tm tmp{};
    std::istringstream ss("1900_1_1_" + fileName);
    ss >> std::get_time(&tmp, "%Y_%m_%d_%H_%M_%S");
    return std::mktime(&tmp);
}

struct VideoReplaySettings final {
    std::string path;
    std::string identifier;
    std::string cameraName;
    double fps;
    double fov;
    uint32_t width;
    uint32_t height;
};

template <class Inspector>
bool inspect(Inspector& f, VideoReplaySettings& x) {
    return f.object(x).fields(
        f.field("path", x.path), f.field("identifier", x.identifier).fallback(""),
        f.field("cameraName", x.cameraName).fallback("Video"),
        f.field("fps", x.fps).fallback(30.0).invariant([](const double v) { return v >= 1.0 && v <= 120.0; }),
        f.field("fov", x.fov), f.field("width", x.width), f.field("height", x.height));
}

class VideoReplay final : public HubHelper<caf::event_based_actor, VideoReplaySettings, image_frame_atom> {
private:
    cv::VideoCapture mCapture;
    Identifier mKey;
    bool isDirectory;
    uint32_t mVideoIndex;
    std::vector<std::string> videoPaths;

    cv::Mat cameraMatrix, distCoefficients;
    cv::Mat resize(const cv::Mat& frame) const {
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size{ static_cast<int32_t>(mConfig.width), static_cast<int32_t>(mConfig.height) });
        return resized;
    }

    void getDirectoryVideos(const std::string& directoryPath) {
        for(const auto& file : fs::directory_iterator(directoryPath))
            videoPaths.push_back(file.path());
        std::sort(videoPaths.begin(), videoPaths.end(), [](const std::string& lhs, const std::string& rhs) {
            return std::difftime(parseTimePoint(lhs), parseTimePoint(rhs)) < 0;
        });
    }

    std::string getNextPath() {
        if(mVideoIndex >= videoPaths.size())
            mVideoIndex = 0;
        logInfo(fmt::format("Current play: {}", videoPaths[mVideoIndex]));
        return videoPaths[mVideoIndex++];
    }

    void next() {
        cv::Mat img;
        if(!mCapture.read(img)) {
            mCapture.release();
            if(isDirectory) {
                mCapture.open(getNextPath());
            } else {
                mCapture.open(mConfig.path);
            }
            return;
        }

        CameraFrame res;
        res.frame = (mConfig.width == static_cast<uint32_t>(img.cols) && mConfig.height == static_cast<uint32_t>(img.rows)) ?
            std::move(img) :
            resize(img);
        res.info.width = mConfig.width;
        res.info.height = mConfig.height;
        if(mConfig.identifier.empty()) {
            res.info.identifier = "VideoReplay";
            res.info.cameraMatrix =
                (cv::Mat_<double>(3, 3) << static_cast<float>(mConfig.width) / 2 / std::tan(glm::radians(mConfig.fov) / 2), 0,
                 mConfig.width / 2, 0, static_cast<float>(mConfig.height) / 2 / std::tan(glm::radians(mConfig.fov) / 2),
                 mConfig.height / 2, 0, 0, 1);
            res.info.distCoefficients = cv::Mat_<double>{};
        } else {
            res.info.identifier = mConfig.identifier;
            res.info.cameraMatrix = cameraMatrix;
            res.info.distCoefficients = distCoefficients;
        }

        res.info.tfGun2Camera = glm::identity<glm::dmat4>();
        res.info.tfRobot2Gun = glm::identity<glm::dmat4>();
        res.lastUpdate = SynchronizedClock::instance().now();

#ifdef ARTINX_RADAR
        sendAll(image_frame_atom_v,
                BlackBoard::instance().updateSync(mKey, std::move(res), std::string_view(mConfig.cameraName)));
#else
        sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res), std::string_view("VideoReplay")));
#endif
    }

public:
    VideoReplay(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ generateKey(this) }, mVideoIndex(0) {
        isDirectory = fs::is_directory(mConfig.path);
        if(isDirectory) {
            if(fs::is_empty(mConfig.path)) {
                throw std::runtime_error(fmt::format("The directory is empty", mConfig.path));
            } else {
                getDirectoryVideos(mConfig.path);
            }
        } else {
            if(!mCapture.open(mConfig.path)) {
                const auto error = "Failed to load video " + mConfig.path;
                logError(error.c_str());
            }
        }

        if(mConfig.identifier.empty()) {
        } else {
            const auto inputFileName = "./data/camera_calibration/" + mConfig.identifier + ".xml";
            const cv::FileStorage fs(inputFileName, cv::FileStorage::READ);
            if(std::filesystem::exists(inputFileName) && fs.isOpened()) {
                fs["camera_matrix"] >> cameraMatrix;
                fs["distortion_coefficients"] >> distCoefficients;
            } else
                logError("Failed to get calibration info for S/N " + mConfig.identifier);
        }
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                    Timer::instance().addTimer(address(),
                                               std::chrono::microseconds{ static_cast<int64_t>(1'000'000 / mConfig.fps) });
                },
                 [this](timer_atom) {
                     ACTOR_PROTOCOL_CHECK(timer_atom);
                     next();
                 } };
    }
};

HUB_REGISTER_CLASS(VideoReplay);
