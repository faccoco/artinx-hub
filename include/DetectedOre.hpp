#pragma once
#include "CameraFrame.hpp"
#include <opencv2/opencv.hpp>

enum class MoveDirection { LEFT, RIGHT, STAY, INVALID };

struct Movement final {
    MoveDirection direction;
    double distance;
};

struct OrePosition {
    uint64_t totalNum;
    uint64_t flashingIndex;  // begin with 0
};
struct DetectedOreArray final {
    CameraFrame frame;
    Movement currentMovement;
    std::vector<OrePosition> orePositionHistory;
};
