#pragma once
#include "DetectedTarget.hpp"
#include "Timer.hpp"
#include <optional>

struct SelectedTarget final {
    TimePoint lastUpdate;
    std::optional<Transform<FrameOfRef::Robot, FrameOfRef::Gun, true>> tfRobot2Gun;
    std::optional<DetectedTarget> selected;
};

struct PredictedTarget final{
    TimePoint lastUpdate;
    Vector<UnitType::Distance, FrameOfRef::Robot> position;
    Vector<UnitType::LinearVelocity, FrameOfRef::Robot> velocity;
};

ACTOR_PROTOCOL_DEFINE(set_target_atom, TypedIdentifier<SelectedTarget>);
ACTOR_PROTOCOL_DEFINE(predict_success_atom, TypedIdentifier<PredictedTarget>);
