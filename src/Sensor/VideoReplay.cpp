#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <opencv2/videoio.hpp>

struct VideoReplaySettings final {
    std::string path;
    double fps;
};

template <class Inspector>
bool inspect(Inspector& f, VideoReplaySettings& x) {
    return f.object(x).fields(f.field("path", x.path),
                              f.field("fps", x.fps).fallback(30.0).invariant([](double v) { return v >= 1.0 && v <= 120.0; }));
}

class VideoReplay final : public HubHelper<caf::event_based_actor, VideoReplaySettings, image_frame_atom> {
private:
    Identifier mKey;
    std::thread mThread;
    bool mStartFlag = false, mRunFlag = true;

public:
    VideoReplay(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(VideoReplay).hash_code() } {
        mThread = std::thread{ [this] {
            const auto [path, fps] = mConfig;
            while(!mStartFlag)
                std::this_thread::sleep_for(100ms);

            while(mRunFlag) {
                cv::VideoCapture capture;
                if(!capture.open(path)) {
                    // TODO: error code
                    this->quit(caf::error{});
                }

                const auto duration = std::chrono::floor<Clock::duration>(static_cast<Clock::duration>(1s) / fps);
                auto now = Clock::now();
                while(true) {
                    std::this_thread::sleep_until(now);

                    CameraFrame frameData;
                    frameData.lastUpdate = now;
                    // TODO: frameData.info

                    if(!capture.read(frameData.frame)) {
                        break;
                    }

                    BlackBoard::instance().updateSync(mKey, std::move(frameData));

                    sendAll(image_frame_atom_v, mKey);
                    now += duration;
                }
            }
        } };
    }
    ~VideoReplay() {
        mRunFlag = false;
        mThread.join();
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) { mStartFlag = true; } };
    }
};

HUB_REGISTER_CLASS(VideoReplay);
