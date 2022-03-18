#pragma once
#include "CameraFrame.hpp"
#include <opencv2/opencv.hpp>

struct DetectedCarArray final {
    CameraFrame frame;
    std::vector<cv::Rect> cars;
};
