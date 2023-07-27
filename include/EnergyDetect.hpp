#pragma once
#include "CameraFrame.hpp"

struct EnergyFan final {
    CameraFrame frame;
    std::vector<cv::Point> keyPoints; // for drawer
};

ACTOR_PROTOCOL_DEFINE(energy_detect_available_atom, TypedIdentifier<EnergyFan>);
ACTOR_PROTOCOL_DEFINE(energy_detector_control_atom, bool);
