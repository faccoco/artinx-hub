#pragma once
#include "CameraFrame.hpp"
#include "Timer.hpp"

struct RadarCameraPoints final {
    CameraInfo info;
    std::vector<cv::Point2f> points;
};

struct RadarTransform final {
    glm::dmat4 trans;
    glm::dmat3 rotate;
};

ACTOR_PROTOCOL_DEFINE(radar_locate_request_atom, TypedIdentifier<RadarCameraPoints>);
ACTOR_PROTOCOL_DEFINE(radar_locate_succeed_atom, TypedIdentifier<Transform<FrameOfRef::Camera, FrameOfRef::Ground, true>>);
ACTOR_PROTOCOL_DEFINE(radar_locate_succeed_atom, TypedIdentifier<RadarTransform>);
