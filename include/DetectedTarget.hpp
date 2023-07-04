#pragma once
#include "DetectedArmor.hpp"
#include "Timer.hpp"
#include "Transform.hpp"
#include <vector>

#include <opencv2/opencv.hpp>

enum class ArmorMotion { Unsure, Static, Moving };

struct DetectedTarget final {
    cv::Point2f armorImgCenter;
    double distToImgCenter;     // 距离图像中心的距离（策略将会优先击打距离图像中心近的装甲板）
    Point<UnitType::Distance, FrameOfRef::Gun> center;
    RobotType id;
    ArmorType type;
    ArmorMotion motion;     //get from strategy
    Transform<FrameOfRef::Armor, FrameOfRef::Gun> rmat;
};

struct DetectedTargetArray final {
    TimePoint lastUpdate;
    Transform<FrameOfRef::Robot, FrameOfRef::Gun, true> tfRobot2Gun;
    std::vector<DetectedTarget> targets;
};

ACTOR_PROTOCOL_DEFINE(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
