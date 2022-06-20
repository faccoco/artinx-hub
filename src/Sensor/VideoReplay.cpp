#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <glm/gtc/matrix_transform.hpp>
#include <opencv2/videoio.hpp>

struct VideoReplaySettings final {
    std::string path;
    double fps;
    double fov;
    uint32_t width;
    uint32_t height;
};

template <class Inspector>
bool inspect(Inspector& f, VideoReplaySettings& x) {
    return f.object(x).fields(f.field("path", x.path), f.field("fps", x.fps).fallback(30.0).invariant([](const double v) {
        return v >= 1.0 && v <= 120.0;
    }),
                              f.field("fov", x.fov), f.field("width", x.width), f.field("height", x.height));
}

class VideoReplay final : public HubHelper<caf::event_based_actor, VideoReplaySettings, image_frame_atom> {
private:
    cv::VideoCapture mCapture;
    Identifier mKey;

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
        res.frame = (mConfig.width == img.cols && mConfig.height == img.rows) ? std::move(img) : resize(img);
        res.info.width = mConfig.width;
        res.info.height = mConfig.height;
        res.info.identifier = "VideoReplay";
        res.info.cameraMatrix =
            (cv::Mat_<double>(3, 3) << mConfig.width / 2 / tan(glm::radians(mConfig.fov) / 2), 0, mConfig.width / 2, 0,
             mConfig.height / 2 / tan(glm::radians(mConfig.fov) / 2), mConfig.height / 2, 0, 0, 1);
        res.info.distCoefficients = cv::Mat_<double>{};
        res.info.transform = Transform<FrameOfReference::Gun, FrameOfReference::Camera, true>(glm::identity<glm::dmat4>());
        res.lastUpdate = SynchronizedClock::instance().now();

        BlackBoard::instance().updateSync(mKey, std::move(res));
        sendAll(image_frame_atom_v, mKey);
    }

public:
    VideoReplay(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(VideoReplay).hash_code() } {

        if(!mCapture.open(mConfig.path)) {
            const auto error = "Failed to load video " + mConfig.path;
            logError(error.c_str());
        }
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    Timer::instance().addTimer(address(),
                                               std::chrono::microseconds{ static_cast<int64_t>(1'000'000 / mConfig.fps) });
                },
                 [this](timer_atom) { next(); } };
    }
};

HUB_REGISTER_CLASS(VideoReplay);
