#pragma once
#include "CameraFrame.hpp"
#include "DataDesc.hpp"

#include "SuppressWarningBegin.hpp"

#include <opencv2/opencv.hpp>

#include "SuppressWarningEnd.hpp"

enum RobotType {
    Sentry = 0,
    Hero = 1,
    Engineer = 2,
    Infantry1 = 3,
    Infantry2 = 4,
    Infantry3 = 5,
    Outpost = 6,
    Base = 7,
    Negative = 8
};

enum class ArmorType { Small, Large };

struct PairedLight final {
    cv::RotatedRect r1;
    cv::RotatedRect r2;
};

struct Light final {
    Color color;
    cv::Point2f top, bottom, center;
    float length;
    float width;
    float ratio;
    float tiltAngle;
};

struct Armor final {
    std::vector<cv::Point2f> light4Point;  // 灯条四点坐标
    cv::Rect2f lightRect;                  // 灯条四点矩形
    RobotType robotType;  // 机器人类别（0：哨兵，1：英雄，2：工程，3、4、5：步兵，6：前哨站，7：基地）
    Color robotColor;  // 颜色分类（0：蓝色，1：红色，2：灰色）
    bool isLargeArmor;
    float prob;  // 分类置信度
};

struct DetectedArmorArray final {
    CameraFrame frame;
    std::vector<Armor> armors;
};

ACTOR_PROTOCOL_DEFINE(armor_detect_available_atom, TypedIdentifier<DetectedArmorArray>);