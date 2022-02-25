#pragma once
#include "CameraFrame.hpp"
#include "Timer.hpp"
#include "Transform.hpp"
#include <opencv2/opencv.hpp>

struct DetectedEnergyArray final {
    CameraFrame frame;
    bool direction;
    cv::Point2f predictPoint;
};