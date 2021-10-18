#pragma once
#include "Timer.hpp"
#include "Transform.hpp"

struct HeadInfo final {
    TimePoint lastUpdate;
    Transform<FrameOfReference::Robot, FrameOfReference::Gun, true> transform;
    double yawSpeed;
    double pitchSpeed;
};
