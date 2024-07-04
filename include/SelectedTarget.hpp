#pragma once
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedTarget.hpp"
#include "Timer.hpp"
#include <glm/fwd.hpp>
#include <optional>

struct SelectedTarget final {
    TimePoint lastUpdate;
    Transform<FrameOfRef::Robot, FrameOfRef::Camera, true> tfRobot2Camera;
    std::vector<DetectedTarget> targets;
    std::optional<DetectedTarget> selected;
};

struct PredictedTarget final {
    TimePoint lastUpdate;
    CameraFrame frame;
    Transform<FrameOfRef::Robot, FrameOfRef::Camera, true> tfRobot2Camera;
    Vector<UnitType::Distance, FrameOfRef::Robot> center;
    Scalar<UnitType::Angle> yaw;
    Vector<UnitType::LinearVelocity, FrameOfRef::Robot> linearVel;
    Scalar<UnitType::AngularVelocity> angularVel;
    std::pair<double, double> radius;
    std::pair<double, double> y;
    int armorNum;
    RobotType robotType;
    ArmorType armorType;
};

struct PredictedPeriodTarget final {
    TimePoint lastUpdate;
    std::optional<Vector<UnitType::Distance, FrameOfRef::Robot>> position;
    std::optional<double> period;
};

struct SelectedTargetInfo final {
    TimePoint lastUpdate;
    double yawAngle;
    double pitchAngle;
    bool isFire;
    ArmorType armorType;
    SolverType solveType;
    std::optional<glm::dvec3> targetPos;
    std::optional<RobotType> targetType;
};

struct ProjectedTarget final {
    TimePoint lastUpdate;
    std::vector<cv::Point2d> armorCorners;
    std::pair<std::vector<cv::Point2d>, std::vector<cv::Point2d>> projectedPoints;
};

ACTOR_PROTOCOL_DEFINE(hero_strategy_control_atom, bool, bool);
ACTOR_PROTOCOL_DEFINE(set_target_atom, TypedIdentifier<SelectedTarget>);
ACTOR_PROTOCOL_DEFINE(set_period_target_atom, TypedIdentifier<SelectedTarget>, bool);
ACTOR_PROTOCOL_DEFINE(set_period_outpost_atom, TypedIdentifier<SelectedTarget>, bool);
ACTOR_PROTOCOL_DEFINE(predict_success_atom, TypedIdentifier<PredictedTarget>);
ACTOR_PROTOCOL_DEFINE(car_predict_atom, TypedIdentifier<PredictedTarget>);
ACTOR_PROTOCOL_DEFINE(period_predict_success_atom, TypedIdentifier<PredictedPeriodTarget>);
ACTOR_PROTOCOL_DEFINE(set_target_info_atom, TypedIdentifier<SelectedTargetInfo>);
ACTOR_PROTOCOL_DEFINE(angle_solver_view_atom, TypedIdentifier<ProjectedTarget>);
