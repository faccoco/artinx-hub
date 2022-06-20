#pragma once
#include "CameraFrame.hpp"
#include <opencv2/opencv.hpp>

struct DetectedEnergyInfo final {
    TimePoint lastUpdate;
    Point<UnitType::Distance, FrameOfReference::Gun> point;
    Point<UnitType::Distance, FrameOfReference::Gun> prePoint;
};

ACTOR_PROTOCOL_DEFINE(energy_detect_available_atom, TypedIdentifier<DetectedEnergyInfo>);
ACTOR_PROTOCOL_DEFINE(energy_detector_control_atom, bool, int);
