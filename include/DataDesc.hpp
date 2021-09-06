#pragma once
#include <caf/allowed_unsafe_message_type.hpp>
#include <caf/type_id.hpp>
#include <opencv2/opencv.hpp>

struct VideoReplaySettings final {
    std::string path;
    double fps;
};

template <class Inspector>
bool inspect(Inspector& f, VideoReplaySettings& x) {
    return f.object(x).fields(f.field("path", x.path),
                              f.field("fps", x.fps).fallback(30.0).invariant([](double v) { return v >= 1.0 && v <= 120.0; }));
}

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

struct DahengDriverSettings final {
    std::string serialNumber;
    double fps;
    uint32_t width;
    uint32_t height;
};

template <class Inspector>
bool inspect(Inspector& f, DahengDriverSettings& x) {
    return f.object(x).fields(f.field("serialNumber", x.serialNumber),
                              f.field("fps", x.fps).fallback(30.0).invariant([](double v) { return v >= 1.0 && v <= 500.0; }),
                              f.field("width", x.width), f.field("height", x.height));
}

CAF_BEGIN_TYPE_ID_BLOCK(ArtinxHub, caf::first_custom_type_id)

CAF_ADD_ATOM(ArtinxHub, start_atom)
CAF_ADD_TYPE_ID(ArtinxHub, (cv::Mat))
CAF_ADD_TYPE_ID(ArtinxHub, (VideoReplaySettings))
CAF_ADD_TYPE_ID(ArtinxHub, (VideoRecorderSettings))
CAF_ADD_TYPE_ID(ArtinxHub, (DahengDriverSettings))

CAF_END_TYPE_ID_BLOCK(ArtinxHub)

CAF_ALLOW_UNSAFE_MESSAGE_TYPE(cv::Mat)
