#pragma once
#include "CameraFrame.hpp"
#include "DetectedTarget.hpp"

#include "SuppressWarningBegin.hpp"

#include <opencv2/opencv.hpp>

#include "SuppressWarningEnd.hpp"

struct Light final {
    COLOR color;
    cv::Point2f top, bottom;
    double length;
    double width;
    float tiltAngle;
}

struct Armor final {
    std::string id;
    Light leftLight, rightLight;
    cv::Point2f center;
    float confidence;
    ArmorType armorType;
};

struct DetectedArmorArray final {
    CameraFrame frame;
    std::vector<Armor> armors;
};

struct NNetDetectedArmor final {
    std::vector<cv::Point2f> light4Point;  // 灯条四点坐标
    cv::Rect2f lightRect;                  // 灯条四点矩形
    int robotType;  // 机器人类别（0：哨兵，1：英雄，2：工程，3、4、5：步兵，6：前哨站，7：基地）
    int robotColor;                     // 颜色分类（0：蓝色，1：红色，2：灰色）
    float prob;                         // 分类置信度
    std::vector<cv::Point2f> armorPts;  // 探测到的一块装甲板可能的灯条的四点坐标
};

struct NNetDetectedArmorArray final {
    std::vector<NNetDetectedArmor> armors;
    CameraFrame frame;
};

ACTOR_PROTOCOL_DEFINE(armor_detect_available_atom, TypedIdentifier<DetectedArmorArray>);
ACTOR_PROTOCOL_DEFINE(armor_nnet_detect_available_atom, TypedIdentifier<NNetDetectedArmorArray>);
