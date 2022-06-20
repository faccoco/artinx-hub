#pragma once
#include "DetectedTarget.hpp"
#include "Timer.hpp"
#include <optional>

struct SelectedTarget final {
    TimePoint lastUpdate;
    std::optional<DetectedTarget> selected;
};

ACTOR_PROTOCOL_DEFINE(set_target_atom, TypedIdentifier<SelectedTarget>);
