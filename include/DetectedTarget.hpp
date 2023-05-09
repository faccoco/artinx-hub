#pragma once
#include "Timer.hpp"
#include "Transform.hpp"
#include <vector>

#include <opencv2/opencv.hpp>

enum class ArmorType { Small, Large };

struct DetectedTarget final {
    cv::Point2f armorImgCenter;
    double distanceToImgCenter;
    Point<UnitType::Distance, FrameOfRef::Gun> center;
    double area;
    int32_t id;
    ArmorType type;
    Transform<FrameOfRef::Armor, FrameOfRef::Gun> rmat;
};

struct DetectedTargetArray final {
    TimePoint lastUpdate;
    Transform<FrameOfRef::Robot, FrameOfRef::Gun, true> tfRobot2Gun;
    std::vector<DetectedTarget> targets;
};

ACTOR_PROTOCOL_DEFINE(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
