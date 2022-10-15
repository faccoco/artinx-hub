#pragma once
#include "CameraFrame.hpp"

#include "SuppressWarningBegin.hpp"

#include <opencv2/opencv.hpp>

#include "SuppressWarningEnd.hpp"

// Origin: left-top corner of the car's ROI
struct PairedLight final {
    cv::RotatedRect r1;
    cv::RotatedRect r2;
};

struct Armor final {
    int id;
    PairedLight pairedLight;
};

struct DetectedArmorArray final {
    CameraFrame frame;
    std::vector<Armor> armors;
};

ACTOR_PROTOCOL_DEFINE(armor_detect_available_atom, TypedIdentifier<DetectedArmorArray>);
