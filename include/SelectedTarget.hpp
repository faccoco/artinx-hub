#pragma once
#include "DetectedTarget.hpp"
#include "Timer.hpp"
#include <optional>

enum class PredictorType { Car, Outpost, Period, PeriodOutpost };

typedef uchar SolverType;
static constexpr SolverType solverType_normal = 0, solverType_outpost = 1, solverType_period = 2;

struct SelectedTarget final {
    TimePoint lastUpdate;
    std::optional<Transform<FrameOfRef::Robot, FrameOfRef::Gun, true>> tfRobot2Gun;
    std::optional<DetectedTarget> selected;
};

struct TargetROI final {
    TimePoint lastUpdate;
    double dist;
    cv::Point2f armorImgCenter;
};

struct PredictedTarget final {
    TimePoint lastUpdate;
    Vector<UnitType::Distance, FrameOfRef::Robot> position;
    Vector<UnitType::LinearVelocity, FrameOfRef::Robot> velocity;
};

struct PredictedOutpost final {
    TimePoint lastUpdate;
    Vector<UnitType::Distance, FrameOfRef::Robot> centerOfOutpost;
    Scalar<UnitType::Distance> radius;
    Scalar<UnitType::Angle> theta;
    Scalar<UnitType::AngularVelocity> angularVelocity;
};

struct PredictedPeriodTarget final {
    TimePoint lastUpdate;
    Vector<UnitType::Distance, FrameOfRef::Robot> position;
    std::optional<double> period;
};

ACTOR_PROTOCOL_DEFINE(outpost_detector_control_atom, bool);
ACTOR_PROTOCOL_DEFINE(update_roi_atom, TypedIdentifier<TargetROI>);
ACTOR_PROTOCOL_DEFINE(set_target_atom, TypedIdentifier<SelectedTarget>);
ACTOR_PROTOCOL_DEFINE(set_outpost_atom, TypedIdentifier<SelectedTarget>);
ACTOR_PROTOCOL_DEFINE(set_period_target_atom, TypedIdentifier<SelectedTarget>, bool);
ACTOR_PROTOCOL_DEFINE(set_period_outpost_atom, TypedIdentifier<SelectedTarget>, bool);
ACTOR_PROTOCOL_DEFINE(predict_success_atom, TypedIdentifier<PredictedTarget>);
ACTOR_PROTOCOL_DEFINE(outpost_predict_success_atom, TypedIdentifier<PredictedOutpost>);
ACTOR_PROTOCOL_DEFINE(period_predict_success_atom, TypedIdentifier<PredictedPeriodTarget>);
ACTOR_PROTOCOL_DEFINE(set_target_info_atom, GroupMask, Clock::rep, double, double, bool, SolverType);
