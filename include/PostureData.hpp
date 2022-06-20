#pragma once
#include "Timer.hpp"
#include "Transform.hpp"

struct PostureData final {
    TimePoint lastUpdate;

    Transform<FrameOfReference::Ground, FrameOfReference::Robot> postureOfRobot;

    Vector<UnitType::LinearVelocity, FrameOfReference::Ground> linearVelocityOfRobot;
    Vector<UnitType::AngularVelocity, FrameOfReference::Ground> angularVelocityOfRobot;
    Vector<UnitType::LinearAcceleration, FrameOfReference::Ground> linearAccelerationOfRobot;
    Vector<UnitType::AngularAcceleration, FrameOfReference::Ground> angularAccelerationOfRobot;
};

ACTOR_PROTOCOL_DEFINE(update_posture_atom, TypedIdentifier<PostureData>);
