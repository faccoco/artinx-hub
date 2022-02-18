#pragma once
#include "CameraFrame.hpp"
#include "Timer.hpp"
#include <opencv2/opencv.hpp>

enum class Color { Red, Blue };

struct RadarCameraPointsArray final {
    TimePoint lastUpdate;
    CameraInfo cameraInfo;
    Color selfColor;
    std::vector<cv::Point2f> imagePoints;
};
