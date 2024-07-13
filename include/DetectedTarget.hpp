#pragma once
#include "DetectedArmor.hpp"
#include "Timer.hpp"
#include "Transform.hpp"
#include <vector>

#include <opencv2/opencv.hpp>

enum class ArmorMotion { Unsure, Static, Moving };

struct DetectedTarget final {
    cv::Point2f armorImgCenter;
    double distToImgCenter;  // 距离图像中心的距离（策略将会优先击打距离图像中心近的装甲板）
    Point<UnitType::Distance, FrameOfRef::Camera> center;
    RobotType id;
    ArmorType type;
    ArmorMotion motion;  // get from strategy
    std::vector<cv::Point2f> armorPoints;
    Transform<FrameOfRef::Armor, FrameOfRef::Camera> rmat;
};

struct DetectedTargetArray final {
    TimePoint lastUpdate;
    Transform<FrameOfRef::Robot, FrameOfRef::Camera, true> tfRobot2Camera;
    std::vector<DetectedTarget> targets;
};

ACTOR_PROTOCOL_DEFINE(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
