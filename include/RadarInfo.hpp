#pragma once
#ifdef ARTINX_RADAR
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Timer.hpp"
#include "Transform.hpp"
#include "yolo.hpp"
#include <atomic>
#include <glm/fwd.hpp>
#include <mutex>
#include <opencv2/core.hpp>
#include <shared_mutex>

struct RadarCameraPoints final {
    CameraInfo frame;
    std::vector<cv::Point2f> points;
};

struct RadarTransform final {
    glm::dmat4 trans{};
    std::shared_mutex mutex;
    std::atomic<bool> flag;

    RadarTransform() : flag(false) {}

    void setReady();
    bool isReady();
    glm::dmat4 load();
    void store(const glm::dmat4& rhs);
    static RadarTransform& instant();
};

struct RadarPerspectiveTransform final {
    cv::Mat trans;
    std::shared_mutex mutex;
    std::atomic<bool> flag;

    RadarPerspectiveTransform() : flag(false) {}

    void setReady();
    bool isReady();
    cv::Mat load();
    void store(const cv::Mat& rhs);
    static RadarPerspectiveTransform& instant();
};

struct DetectedBotPosition final {
    uint16_t id;
    double x;
    double y;
};

struct BotsPosition final {
    std::vector<DetectedBotPosition> data;
};

struct DetectedBots {
    CameraFrame frame;
    std::vector<yolo::Box> botBoxes;
};

struct ReadableBotTag {
    Color color;
    uint8_t index;
};

ACTOR_PROTOCOL_DEFINE(update_radar_atom);
ACTOR_PROTOCOL_DEFINE(radar_locate_request_atom, TypedIdentifier<RadarCameraPoints>);
ACTOR_PROTOCOL_DEFINE(bots_locate_request_atom, TypedIdentifier<DetectedBots>);
ACTOR_PROTOCOL_DEFINE(sync_position_atom, TypedIdentifier<BotsPosition>);
#endif
