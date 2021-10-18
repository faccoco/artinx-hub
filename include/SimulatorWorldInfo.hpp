#pragma once
#include "Timer.hpp"
#include "Transform.hpp"
#include <vector>

struct SimulatorWorldInfo final {
    TimePoint lastUpdate;

    Transform<FrameOfReference::Ground, FrameOfReference::Robot, true> posture;

    std::vector<Point<UnitType::Distance, FrameOfReference::Ground>> targets;
};
