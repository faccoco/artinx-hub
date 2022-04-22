#pragma once
#include "CameraFrame.hpp"
#include <opencv2/opencv.hpp>

enum class OreDetectorMode { GOLD_OVER, GOLD_GROUND, SILVER_GROUND, NONE };

struct OrePosition {
    // begin with 0
    uint64_t totalNum;
    uint64_t flashingIndex;
};

struct OreAlignmentMessage final {
    CameraFrame frame;
    OreDetectorMode detectMode;
    OreDetectorMode lastMode;
    double moveDistance;
};
