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

struct DetectedArmorsOfCar final {
    cv::Rect roi;
    int id;
    std::vector<PairedLight> armors;
};

struct DetectedArmorArray final {
    CameraFrame frame;
    std::vector<DetectedArmorsOfCar> armors;
};

ACTOR_PROTOCOL_DEFINE(armor_detect_available_atom, TypedIdentifier<DetectedArmorArray>);
