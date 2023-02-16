#pragma once
#include "Timer.hpp"
#include "Transform.hpp"
#include <variant>

#include "SuppressWarningBegin.hpp"

#include <opencv2/opencv.hpp>

#include "SuppressWarningEnd.hpp"

struct CameraInfo final {
    Transform<FrameOfRef::Gun, FrameOfRef::Camera, true> tfGun2Camera;
    std::optional<Transform<FrameOfRef::Robot, FrameOfRef::Gun, true>> tfRobot2Gun;
    std::string identifier;
    cv::Mat cameraMatrix;
    cv::Mat distCoefficients;
    uint32_t width;
    uint32_t height;
};

struct CameraFrame final {
    TimePoint lastUpdate;
    CameraInfo info;
    cv::Mat frame;
};

ACTOR_PROTOCOL_DEFINE(image_frame_atom, TypedIdentifier<CameraFrame>);
ACTOR_PROTOCOL_DEFINE(image_frame_atom, TypedIdentifier<CameraFrame, std::string_view>);
