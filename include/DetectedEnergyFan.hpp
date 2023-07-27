#pragma once
#include "CameraFrame.hpp"

struct EnergyFan final {
    TimePoint lastUpdate;
    CameraInfo cameraInfo;
    std::vector<cv::Point2f> keyPoints;
};

ACTOR_PROTOCOL_DEFINE(energy_detect_available_atom, TypedIdentifier<EnergyFan>);
ACTOR_PROTOCOL_DEFINE(energy_detector_control_atom, uint8_t, double);
