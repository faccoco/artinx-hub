#pragma once

#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Timer.hpp"
#include "Transform.hpp"

struct RadarCameraPoints final {
    CameraInfo info;
    std::vector<cv::Point2f> points;
};

struct RadarTransform final {
private:
    RadarTransform() = default;

public:
    Transform<FrameOfRef::Camera, FrameOfRef::Ground> trans;
    static RadarTransform& Instance();
};

struct DetectedBotPosition final {
    uint16_t id;
    double x, y, z;
};

struct BotsPosition final {
    std::vector<DetectedBotPosition> data;
};

ACTOR_PROTOCOL_DEFINE(update_radar_atom);
ACTOR_PROTOCOL_DEFINE(radar_locate_request_atom, TypedIdentifier<RadarCameraPoints>);
ACTOR_PROTOCOL_DEFINE(radar_locate_succeed_atom);
ACTOR_PROTOCOL_DEFINE(sync_position_atom, TypedIdentifier<BotsPosition>);
