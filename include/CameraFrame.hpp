#pragma once
#include "Timer.hpp"
#include "Transform.hpp"
#include <opencv2/opencv.hpp>
#include <variant>

struct CameraInfo final {
    std::variant<Transform<FrameOfReference::Gun, FrameOfReference::Camera, true>,
                 Transform<FrameOfReference::Robot, FrameOfReference::Camera, true>>
        transform;
    std::string identifier;
    cv::Mat cameraMatrix;
    cv::Mat distCoefficients; // TODO: unused
    uint32_t width;
    uint32_t height;
};

struct CameraFrame final {
    TimePoint lastUpdate;
    CameraInfo info;
    cv::Mat frame;
};

ACTOR_PROTOCOL_DEFINE(image_frame_atom, TypedIdentifier<CameraFrame>);
