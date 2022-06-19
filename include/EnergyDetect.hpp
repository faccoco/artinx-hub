#pragma once
#include "CameraFrame.hpp"
#include <opencv2/opencv.hpp>

struct DetectedEnergyInfo final {
    TimePoint lastUpdate;
    Point<UnitType::Distance, FrameOfReference::Gun> point;
    Point<UnitType::Distance, FrameOfReference::Gun> prePoint;
};
