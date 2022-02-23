#pragma once
#include "Timer.hpp"
#include "Transform.hpp"
#include <vector>

struct DetectedTarget final {
    Point<UnitType::Distance, FrameOfReference::Gun> center;
    double area;
    int32_t id;
    Vector<UnitType::LinearVelocity, FrameOfReference::Gun> velocity;
};

struct DetectedTargetArray final {
    TimePoint lastUpdate;
    std::vector<DetectedTarget> targets;
};
