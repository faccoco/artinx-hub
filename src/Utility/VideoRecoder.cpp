#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <opencv2/videoio.hpp>

namespace fs = std::filesystem;

struct VideoRecorderSettings final {
    std::string base;
    double segmentLength;
    double fps;
};

template <class Inspector>
bool inspect(Inspector& f, VideoRecorderSettings& x) {
    return f.object(x).fields(
        f.field("base", x.base),
        f.field("segmentLength", x.segmentLength).fallback(60.0).invariant([](double v) { return v >= 10.0; }),
        f.field("fps", x.fps).fallback(30.0).invariant([](double v) { return v >= 1.0 && v <= 120.0; }));
}

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
                 [this](image_frame_atom, Identifier key) {
                     if(!mStartFlag)
                         return;

                     const auto frameData = BlackBoard::instance().get<CameraFrame>(key).value();
                     // TODO: record camera info

                     if(mWriter &&
                        (frameData.frame.type() != mFormat || frameData.frame.size() != mSize ||
                         mFrameCount >= static_cast<uint32_t>(mConfig.fps * mConfig.segmentLength))) {
                         mWriter.reset();
                     }
                     if(!mWriter) {
                         if(!fs::create_directories(mConfig.base)) {
                             const auto error = "Failed to create directory " + mConfig.base;
                             CAF_RAISE_ERROR(error.c_str());
                         }
                         mWriter = std::make_unique<cv::VideoWriter>(
                             mConfig.base + "/" + std::to_string(Clock::now().time_since_epoch().count()) + ".mp4", mFourcc,
                             mConfig.fps, frameData.frame.size());
                         mFormat = frameData.frame.type();
                         mSize = frameData.frame.size();
                         mFrameCount = 0;
                     }
                     mWriter->write(frameData.frame);
                     ++mFrameCount;
                 } };
    }
};

HUB_REGISTER_CLASS(VideoRecorder);
