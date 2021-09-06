#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <memory>
#include <opencv2/videoio.hpp>

class VideoRecorder final : public HubHelper<caf::event_based_actor, VideoRecorderSettings> {
private:
    bool mStartFlag = false;
    int mFormat = 0;
    cv::Size mSize = { 0, 0 };
    uint32_t mFrameCount = 0;
    std::unique_ptr<cv::VideoWriter> mWriter;

    const int mFourcc = cv::VideoWriter::fourcc('H', 'E', 'V', 'C');

public:
    VideoRecorder(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    ~VideoRecorder() {
        mWriter.reset();
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) { mStartFlag = true; },
                 [this](cv::Mat image) {
                     if(!mStartFlag || image.empty())
                         return;
                     if(mWriter &&
                        (image.type() != mFormat || image.size() != mSize ||
                         mFrameCount >= static_cast<uint32_t>(mConfig.fps * mConfig.segmentLength))) {
                         mWriter.reset();
                     }
                     if(!mWriter) {
                         mWriter = std::make_unique<cv::VideoWriter>(
                             mConfig.base + "/" + std::to_string(Clock::now().time_since_epoch().count()) + ".mp4", mFourcc,
                             mConfig.fps, image.size());
                         mFormat = image.type();
                         mSize = image.size();
                         mFrameCount = 0;
                     }
                     mWriter->write(image);
                     ++mFrameCount;
                 } };
    }
};

HUB_REGISTER_CLASS(VideoRecorder);
