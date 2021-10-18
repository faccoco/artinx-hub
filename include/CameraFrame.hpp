#pragma once
#include "Timer.hpp"
#include "Transform.hpp"
#include <opencv2/opencv.hpp>
#include <variant>

struct CameraInfo final {
    std::variant<Transform<FrameOfReference::Gun, FrameOfReference::Camera, true>,
                 Transform<FrameOfReference::Robot, FrameOfReference::Camera, true>>
        transform;
    double fov;
    uint32_t width;
    uint32_t height;
};

struct CameraFrame final {
    TimePoint lastUpdate;
    CameraInfo info;
    cv::Mat frame;
};
