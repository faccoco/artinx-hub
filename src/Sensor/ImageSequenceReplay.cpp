#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "Timer.hpp"
#include <caf/event_based_actor.hpp>

struct ImageSequenceReplaySettings final {
    std::string path;
    std::string extension;  // .jpg, .png, etc.
    double fps;
    double fov;
    uint32_t width;
    uint32_t height;
};
template <class Inspector>
bool inspect(Inspector& f, ImageSequenceReplaySettings& x) {
    return f.object(x).fields(
        f.field("path", x.path).invariant([](const std::string& path) { return fs::exists(path) && fs::is_directory(path); }),
        f.field("extension", x.extension),
        f.field("fps", x.fps).fallback(30.0).invariant([](double v) { return v >= 1.0 && v <= 120.0; }), f.field("fov", x.fov),
        f.field("width", x.width), f.field("height", x.height));
}

class ImageSequenceReplay final : public HubHelper<caf::event_based_actor, ImageSequenceReplaySettings, image_frame_atom> {
    Identifier mKey;
    uint32_t mCount = 0;

    cv::Mat resize(const cv::Mat& frame) const {
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size{ static_cast<int32_t>(mConfig.width), static_cast<int32_t>(mConfig.height) });
        return resized;
    }

    void next() {
        const auto path = mConfig.path + '/' + std::to_string(mCount) + mConfig.extension;
        if(!fs::exists(path)) {
            mCount = 0;
            return;
        }

        auto img = cv::imread(path);
        CameraFrame res;
        res.frame = (mConfig.width == img.cols && mConfig.height == img.rows) ? resize(img) : std::move(img);
        res.info.width = mConfig.width;
        res.info.height = mConfig.height;
        res.info.fov = mConfig.fov;
        res.lastUpdate = SynchronizedClock::instance().now();

        BlackBoard::instance().updateSync(mKey, std::move(res));
        sendAll(image_frame_atom_v, mKey);

        ++mCount;
    }

public:
    ImageSequenceReplay(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(ImageSequenceReplay).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    Timer::instance().addTimer(address(),
                                               std::chrono::microseconds{ static_cast<int64_t>(1'000'000 / mConfig.fps) });
                },
                 [this](timer_atom) { next(); } };
    }
};

HUB_REGISTER_CLASS(ImageSequenceReplay);
