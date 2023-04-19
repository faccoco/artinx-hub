#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/videoio.hpp>
#include <string>

#include "SuppressWarningEnd.hpp"

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

    cv::Mat cameraMatrix, distCoefficients;
    cv::Mat resize(const cv::Mat& frame) const {
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size{ static_cast<int32_t>(mConfig.width), static_cast<int32_t>(mConfig.height) });
        return resized;
    }

    void next() {
        cv::Mat img;
        if(!mCapture.read(img)) {
            mCapture.release();
            mCapture.open(mConfig.path);
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
                (cv::Mat_<double>(3, 3) << mConfig.width / 2 / std::tan(glm::radians(mConfig.fov) / 2), 0, mConfig.width / 2, 0,
                 mConfig.height / 2 / std::tan(glm::radians(mConfig.fov) / 2), mConfig.height / 2, 0, 0, 1);
            res.info.distCoefficients = cv::Mat_<double>{};
        } else {
            res.info.identifier = mConfig.identifier;
            res.info.cameraMatrix = cameraMatrix;
            res.info.distCoefficients = distCoefficients;
        }

        res.info.tfGun2Camera = Transform<FrameOfRef::Gun, FrameOfRef::Camera, true>(glm::identity<glm::dmat4>());
        res.lastUpdate = SynchronizedClock::instance().now();

        sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(mKey, res));
#ifdef ARTINX_RADAR
        sendAll(image_frame_atom_v,
                BlackBoard::instance().updateSync(mKey, std::move(res), std::string_view(mConfig.cameraName)));
#else
        sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res), std::string_view("VideoReplay")));
#endif
    }

public:
    VideoReplay(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {

        if(!mCapture.open(mConfig.path)) {
            const auto error = "Failed to load video " + mConfig.path;
            logError(error.c_str());
        }

        if(mConfig.identifier.empty()) {
            logInfo("You haven't input the identifier, make sure your config doesn't use camera inner matrix");
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
