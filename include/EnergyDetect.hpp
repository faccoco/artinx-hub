#pragma once
#include "CameraFrame.hpp"
#include <opencv2/opencv.hpp>

struct DetectedEnergyArray final {
    bool direction;
    cv::Point2f predictPoint;
};
