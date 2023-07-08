#pragma once
#include "DetectedTarget.hpp"
#include "Timer.hpp"
#include <optional>

struct SelectedTarget final {
    TimePoint lastUpdate;
    Transform<FrameOfRef::Robot, FrameOfRef::Gun, true> tfRobot2Gun;
    std::vector<DetectedTarget> targets;
    std::optional<DetectedTarget> selected;
};

struct PredictedTarget final {
    TimePoint lastUpdate;
    Vector<UnitType::Distance, FrameOfRef::Robot> center;
    Scalar<UnitType::Angle> yaw;
    Vector<UnitType::LinearVelocity, FrameOfRef::Robot> linearVel;
    Scalar<UnitType::AngularVelocity> angularVel;
    std::pair<double, double> radius;
    std::pair<double, double> y;
    RobotType id;
};

struct PredictedPeriodTarget final {
    TimePoint lastUpdate;
    std::optional<Vector<UnitType::Distance, FrameOfRef::Robot>> position;
    std::optional<double> period;
};

ACTOR_PROTOCOL_DEFINE(hero_strategy_control_atom, bool, bool);
ACTOR_PROTOCOL_DEFINE(set_target_atom, TypedIdentifier<SelectedTarget>);
ACTOR_PROTOCOL_DEFINE(set_period_target_atom, TypedIdentifier<SelectedTarget>, bool);
ACTOR_PROTOCOL_DEFINE(set_period_outpost_atom, TypedIdentifier<SelectedTarget>, bool);
ACTOR_PROTOCOL_DEFINE(predict_success_atom, TypedIdentifier<PredictedTarget>);
ACTOR_PROTOCOL_DEFINE(car_predict_atom, TypedIdentifier<PredictedTarget>);
ACTOR_PROTOCOL_DEFINE(period_predict_success_atom, TypedIdentifier<PredictedPeriodTarget>);
ACTOR_PROTOCOL_DEFINE(set_target_info_atom, GroupMask, Clock::rep, double, double, bool, SolverType);
