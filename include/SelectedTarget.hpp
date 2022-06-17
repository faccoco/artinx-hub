#pragma once
#include "DetectedTarget.hpp"
#include "Timer.hpp"
#include <optional>

struct SelectedTarget final {
    TimePoint lastUpdate;
    std::optional<DetectedTarget> selected;
};
