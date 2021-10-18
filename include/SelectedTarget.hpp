#pragma once
#include "Timer.hpp"
#include "Transform.hpp"
#include <optional>

struct SelectedTarget final {
    TimePoint lastUpdate;
    std::optional<Point<UnitType::Distance, FrameOfReference::Gun>> center;
};
