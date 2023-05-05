#pragma once
#include "Timer.hpp"
#include "Transform.hpp"
#include <vector>

struct SimulatorWorldInfo final {
    TimePoint lastUpdate;

    Transform<FrameOfRef::Ground, FrameOfRef::Robot, true> tfGround2Robot;

    std::vector<std::pair<Point<UnitType::Distance, FrameOfRef::Ground>, double>> targets;
};

ACTOR_PROTOCOL_DEFINE(simulator_step_atom, TypedIdentifier<SimulatorWorldInfo>);
