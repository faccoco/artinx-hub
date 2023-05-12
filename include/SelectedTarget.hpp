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

struct TargetROI final {
    TimePoint lastUpdate;
    double dist;
    cv::Point2f armorImgCenter;
};

struct PredictedTarget final {
    TimePoint lastUpdate;
    Vector<UnitType::Distance, FrameOfRef::Robot> center;
    Scalar<UnitType::Angle> theta;
    Vector<UnitType::LinearVelocity, FrameOfRef::Robot> lVel;
    Scalar<UnitType::AngularVelocity> aVel;
    std::pair<double, double> radius;
    std::pair<double, double> y;
};

struct PredictedPeriodTarget final {
    TimePoint lastUpdate;
    Vector<UnitType::Distance, FrameOfRef::Robot> position;
    std::optional<double> period;
};

ACTOR_PROTOCOL_DEFINE(outpost_detector_control_atom, bool);
ACTOR_PROTOCOL_DEFINE(update_roi_atom, TypedIdentifier<TargetROI>);
ACTOR_PROTOCOL_DEFINE(set_target_atom, TypedIdentifier<SelectedTarget>);
ACTOR_PROTOCOL_DEFINE(set_period_target_atom, TypedIdentifier<SelectedTarget>, bool);
ACTOR_PROTOCOL_DEFINE(set_period_outpost_atom, TypedIdentifier<SelectedTarget>, bool);
ACTOR_PROTOCOL_DEFINE(predict_success_atom, TypedIdentifier<PredictedTarget>);
ACTOR_PROTOCOL_DEFINE(period_predict_success_atom, TypedIdentifier<PredictedPeriodTarget>);
ACTOR_PROTOCOL_DEFINE(set_target_info_atom, GroupMask, Clock::rep, double, double, bool, SolverType);
