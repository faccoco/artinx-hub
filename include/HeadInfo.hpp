#pragma once
#include "Timer.hpp"
#include "Transform.hpp"

struct HeadInfo final {
    TimePoint lastUpdate;
    Transform<FrameOfRef::Robot, FrameOfRef::Gun, true> transform;
};

ACTOR_PROTOCOL_DEFINE(update_head_atom, GroupMask, TypedIdentifier<HeadInfo>);
