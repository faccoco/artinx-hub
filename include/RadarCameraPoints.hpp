#pragma once
#include "CameraFrame.hpp"
#include "Timer.hpp"

struct RadarCameraPoints final {
    CameraInfo info;
    std::vector<cv::Point2i> points;
};

ACTOR_PROTOCOL_DEFINE(radar_locate_request_atom, TypedIdentifier<RadarCameraPoints>);
ACTOR_PROTOCOL_DEFINE(radar_locate_succeed_atom, TypedIdentifier<Transform<FrameOfRef::Camera, FrameOfRef::Ground, true>>);
