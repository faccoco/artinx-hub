#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedOre.hpp"
#include "Hub.hpp"
#include <algorithm>
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <magic_enum.hpp>
#include <opencv2/opencv.hpp>
#include <utility>

enum class OreDetectorMode { GOLD_OVER_HEAD, GOLD_ON_THE_GROUND };

struct OreAlignmentStettings final {
    OreDetectorMode detectMode;
    std::vector<double> oreHsvLow;
    std::vector<double> oreHsvHigh;
    std::vector<double> lightbarHsvLow;
    std::vector<double> lightbarHsvHigh;
    int32_t minOreArea;
    int32_t maxOreArea;
    int32_t minLightbarArea;
    int32_t maxLightbarArea;
    int32_t altitude;      // between ore and lightbar
    int32_t widthExpand;   // expand search range on width
    int32_t heightExpand;  //...
    int32_t storageFrameCount;
    double flashThreshold;
    double distanceToOre;
    double offset;  // positive when camera is at the right of the car center axis
    double movingThreshold;
};

template <class Inspector>
bool inspect(Inspector& f, OreAlignmentStettings& x) {
    return f.object(x).fields(
        f.field("mode", x.detectMode), f.field("oreHsvLow", x.oreHsvLow), f.field("oreHsvHigh", x.oreHsvHigh),
        f.field("lightbarHsvLow", x.lightbarHsvLow), f.field("lightbarHsvHigh", x.lightbarHsvHigh),
        f.field("minOreArea", x.minOreArea), f.field("minLightbarArea", x.minLightbarArea), f.field("altitude", x.altitude),
        f.field("widthExpand", x.widthExpand), f.field("heightExpand", x.heightExpand),
        f.field("storageFrameCount", x.storageFrameCount), f.field("flashThreshold", x.flashThreshold),
        f.field("distanceToOre", x.distanceToOre), f.field("offset", x.offset), f.field("movingThreshold", x.movingThreshold));
}

class OreAlignment final : public HubHelper<caf::event_based_actor, OreAlignmentStettings, ore_alignment_available_atom> {
    Identifier mKey;

    std::vector<cv::Rect_<int64_t>> oreRectArray;
    cv::Mat bgrFrame, hsvFrame, binaryOreFrame;

    DetectedOreArray solveDirection(DetectedOreArray& oreArray, const OreAlignmentStettings& settings) {
        Movement tempMovement;

        switch(settings.detectMode) {
            case OreDetectorMode::GOLD_OVER_HEAD: {
                detectOres(oreArray, settings);
                modifyOrePreviousArray(oreArray.orePositionHistory, detectLightBar(oreArray.frame.frame, oreRectArray, settings),
                                       settings);
                uint32_t lightOutFrames = 0;
                for(uint64_t i = 0; i != oreArray.orePositionHistory.size(); ++i) {
                    if(oreArray.orePositionHistory[i].flashingIndex != oreArray.orePositionHistory[i].totalNum)
                        ++lightOutFrames;
                }
                if(lightOutFrames / settings.storageFrameCount >= settings.flashThreshold) {
                    for(uint64_t i = oreArray.orePositionHistory.size() - 1; i != -1; --i) {
                        if(oreArray.orePositionHistory[i].flashingIndex != oreArray.orePositionHistory[i].totalNum) {
                            tempMovement.distance = transformToRealDistance(
                                oreRectArray[oreArray.orePositionHistory[i].flashingIndex], oreArray.frame, settings);
                            if(abs(tempMovement.distance) < settings.movingThreshold)
                                tempMovement.direction = MoveDirection::STAY;
                            else if(tempMovement.distance > 0)
                                tempMovement.direction = MoveDirection::RIGHT;
                            else
                                tempMovement.direction = MoveDirection::LEFT;
                        }
                    }
                } else {
                    tempMovement.direction = MoveDirection::STAY;
                    tempMovement.distance = 0.0;
                }
                break;
            }

            case OreDetectorMode::GOLD_ON_THE_GROUND: {
                detectOres(oreArray, settings);
                double minDistance = transformToRealDistance(oreRectArray[0], oreArray.frame, settings);
                for(uint64_t i = 1; i != oreRectArray.size(); ++i) {
                    if(abs(transformToRealDistance(oreRectArray[i], oreArray.frame, settings)) < abs(minDistance))
                        minDistance = abs(transformToRealDistance(oreRectArray[i], oreArray.frame, settings));
                }

                if(abs(minDistance) <= settings.movingThreshold)
                    tempMovement.direction = MoveDirection::STAY;
                else if(minDistance < 0)
                    tempMovement.direction = MoveDirection::LEFT;
                else
                    tempMovement.direction = MoveDirection::RIGHT;
                tempMovement.distance = minDistance;

                break;
            }

            default:
                tempMovement = Movement{ MoveDirection::INVALID, 0.0 };
                break;
        }
        return DetectedOreArray{ oreArray.frame, tempMovement, oreArray.orePositionHistory };
    }

    void detectOres(DetectedOreArray& oreArray, const OreAlignmentStettings& settings) {
        std::vector<std::vector<cv::Point2i>> orePointsArray;

        inRange(hsvFrame, settings.oreHsvLow, settings.oreHsvHigh, binaryOreFrame);

        cv::Mat tempMat = binaryOreFrame.clone();
        dilate(tempMat, binaryOreFrame, std::vector<int32_t>{ 1, 1, 1, 1, 1, 1, 1, 1, 1 });  // todo

        findContours(binaryOreFrame, orePointsArray, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for(const auto& singleMine : orePointsArray) {
            cv::Rect_<int64_t> tempRect = boundingRect(singleMine);
            if(settings.minOreArea < tempRect.area() && tempRect.area() < settings.maxOreArea) {
                cv::rectangle(bgrFrame, tempRect, cv::Scalar(255, 0, 0), 1, cv::LINE_8);
                oreRectArray.push_back(tempRect);
            }
        }
        std::sort(oreRectArray.begin(), oreRectArray.end(), OreAlignment::rectCompare);
    }

    OrePosition detectLightBar(cv::Mat& frame, const std::vector<cv::Rect_<int64_t>>& oreRectArray,
                               const OreAlignmentStettings& settings) {
        cv::Rect tempRect;
        cv::Mat lightbarFrame;
        uint64_t index = 0;
        std::vector<std::vector<cv::Point2i>> lightbarPointsArray;
        for(const auto& oreRect : oreRectArray) {
            cv::Mat cutFrame = frame.rowRange(oreRect.y - settings.heightExpand - settings.altitude,
                                              oreRect.y + oreRect.height + settings.heightExpand - settings.altitude);
            cutFrame = cutFrame.colRange(oreRect.x - settings.widthExpand, oreRect.x + oreRect.width + settings.widthExpand);

            cv::inRange(cutFrame, settings.lightbarHsvLow, settings.lightbarHsvHigh, lightbarFrame);
            cv::findContours(lightbarFrame, lightbarPointsArray, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            for(const auto& singleLightbar : lightbarPointsArray) {
                tempRect = cv::boundingRect(singleLightbar);
                if(settings.minLightbarArea < tempRect.area() && tempRect.area() < settings.maxLightbarArea) {
                    ++index;

                    // draw on frame, could be removed
                    tempRect = cv::Rect(cv::Point2i(tempRect.x + oreRect.x - settings.widthExpand,
                                                    tempRect.y + oreRect.y - settings.heightExpand),
                                        tempRect.size());
                    cv::rectangle(bgrFrame, tempRect, cv::Scalar(0, 255, 0), 1, cv::LINE_8);
                }
            }
        }
        return OrePosition{ oreRectArray.size(), index };
    }

    void modifyOrePreviousArray(std::deque<OrePosition>& orePositionHistory, const OrePosition& currentPosition,
                                const OreAlignmentStettings& settings) {
        if(orePositionHistory.size() < settings.storageFrameCount)
            orePositionHistory.push_back(currentPosition);
        else {
            orePositionHistory.pop_front();
            orePositionHistory.push_back(currentPosition);
        }
    }

    double transformToRealDistance(const cv::Rect_<int64_t>& rect, const CameraFrame& frame,
                                   const OreAlignmentStettings& settings) {
        return (rect.x + rect.width / 2 - frame.info.width) / (frame.info.width / 2 / tan(glm::radians(frame.info.fov))) *
            settings.distanceToOre +
            settings.offset;
    }

    static bool rectCompare(const cv::Rect_<int64_t>& front, const cv::Rect_<int64_t>& back) {
        return front.x < back.x;
    }

public:
    OreAlignment(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(OreAlignment).hash_code() } {}

    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](ore_alignment_available_atom, Identifier key) {
                     const auto settings = BlackBoard::instance().get<OreAlignmentStettings>(key).value();
                     auto data = BlackBoard::instance().get<DetectedOreArray>(key).value();

                     data.frame.frame.convertTo(bgrFrame, CV_32FC3, 1.0 / 255.0);
                     cv::cvtColor(bgrFrame, hsvFrame, cv::COLOR_BGR2HSV_FULL);

                     DetectedOreArray res;
                     res = solveDirection(data, settings);
                     BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(ore_alignment_available_atom_v, mKey);
                 } };
    }
};

HUB_REGISTER_CLASS(OreAlignment);
