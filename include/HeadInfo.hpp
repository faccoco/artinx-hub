#pragma once
#include "Timer.hpp"
#include "Transform.hpp"

struct Pose {
    double roll;
    double pitch;
    double yaw;
};

struct HeadInfo final {
    TimePoint lastUpdate;
    Pose pose;
    Transform<FrameOfRef::Robot, FrameOfRef::Gun, true> tfRobot2Gun;
};
ACTOR_PROTOCOL_DEFINE(update_head_atom, TypedIdentifier<HeadInfo>);
ACTOR_PROTOCOL_DEFINE(update_head_atom, GroupMask, TypedIdentifier<HeadInfo>);
