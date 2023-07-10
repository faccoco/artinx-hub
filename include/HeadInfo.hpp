#pragma once
#include "Timer.hpp"
#include "Transform.hpp"

struct Pose final {
    double roll;
    double pitch;
    double yaw;
};

struct HeadInfo final {
    TimePoint lastUpdate;
    Pose pose;
};
ACTOR_PROTOCOL_DEFINE(update_head_atom, TypedIdentifier<HeadInfo>);
ACTOR_PROTOCOL_DEFINE(update_head_atom, GroupMask, TypedIdentifier<HeadInfo>);
