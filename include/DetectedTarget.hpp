#pragma once
#include "Timer.hpp"
#include "Transform.hpp"
#include "DetectedArmor.hpp"
#include <vector>

#include <opencv2/opencv.hpp>



struct DetectedTarget final {
    cv::Point2f armorImgCenter;
    Point<UnitType::Distance, FrameOfRef::Gun> center;
    double area;
    RobotType id;
    ArmorType type;
};

struct DetectedTargetArray final {
    TimePoint lastUpdate;
    std::optional<Transform<FrameOfRef::Robot, FrameOfRef::Gun, true>> tfRobot2Gun;
    std::vector<DetectedTarget> targets;
};

ACTOR_PROTOCOL_DEFINE(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
