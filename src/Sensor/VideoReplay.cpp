#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <opencv2/videoio.hpp>

class VideoReplay final : public HubHelper<caf::event_based_actor, VideoReplaySettings> {
private:
    std::thread mThread;
    bool mStartFlag;

public:
    VideoReplay(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mStartFlag{ false } {
        mThread = std::thread{ [this] {
            const auto [path, fps] = mConfig;
            while(!mStartFlag)
                std::this_thread::sleep_for(100ms);

            while(true) {
                cv::VideoCapture capture;
                if(!capture.open(path)) {
                    // TODO: error code
                    this->quit(caf::error{});
                }

                const auto duration = std::chrono::floor<Clock::duration>(static_cast<Clock::duration>(1s) / fps);
                auto now = Clock::now();
                while(true) {
                    std::this_thread::sleep_until(now);
                    cv::Mat frame;
                    if(!capture.read(frame)) {
                        break;
                    }
                    sendAll(frame);
                    now += duration;
                }
            }
        } };
    }
    ~VideoReplay() {
        mThread.detach();
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) { mStartFlag = true; } };
    }
};

HUB_REGISTER_CLASS(VideoReplay);
