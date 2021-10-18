#pragma once
#include "CameraFrame.hpp"
#include "Timer.hpp"
#include "Transform.hpp"
#include <opencv2/opencv.hpp>

// Origin: left-top corner of the car's ROI
// Small armor only
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
    TimePoint lastUpdate;
    CameraInfo cameraInfo;
    std::vector<DetectedArmorsOfCar> armors;
};
