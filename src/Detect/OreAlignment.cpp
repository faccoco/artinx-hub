#pragma once
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

struct OreAlignmentSettings final {
    // HSV range
    std::vector<double> goldOreHsvLow;
    std::vector<double> goldOreHsvHigh;
    std::vector<double> silverOreHsvLow;
    std::vector<double> silverOreHsvHigh;
    std::vector<double> lightBarHsvLow;
    std::vector<double> lightBarHsvHigh;
    // area range(pixel,[low value, high value])
    std::vector<uint32_t> overGoldAreaRange;
    std::vector<uint32_t> groundGoldAreaRange;
    std::vector<uint32_t> groundSilverAreaRange;
    std::vector<uint32_t> lightbarAreaRange;

    int32_t altitude;      // between ore and light bar(pixel)
    int32_t widthExpand;   // expand search range on width(pixel)
    int32_t heightExpand;  //...
    int32_t historyFrameCount;
    double flashFrequencyLimit;  // double in [0,1.0]
    double distanceToOre;
    double offset;  // positive when camera is at the right of the car center axis(meter)
    double limitMovementDistance;
};

template <class Inspector>
bool inspect(Inspector& f, OreAlignmentSettings& x) {
    return f.object(x).fields(
        f.field("overGoldAreaRange", x.overGoldAreaRange), f.field("groundGoldAreaRange", x.groundGoldAreaRange),
        f.field("groundSilverAreaRange", x.groundSilverAreaRange), f.field("lightbarAreaRange", x.lightbarAreaRange),
        f.field("altitude", x.altitude), f.field("widthExpand", x.widthExpand), f.field("heightExpand", x.heightExpand),
        f.field("historyFrameCount", x.historyFrameCount), f.field("flashFrequencyLimit", x.flashFrequencyLimit),
        f.field("distanceToOre", x.distanceToOre), f.field("offset", x.offset),
        f.field("limitMovementDistance", x.limitMovementDistance));
}

class OreAlignment final : public HubHelper<caf::event_based_actor, OreAlignmentSettings, ore_alignment_available_atom> {
    Identifier mKey;

    std::vector<cv::Rect_<int64_t>> oreRectArray;
    cv::Mat bgrFrame, hsvFrame, binaryOreFrame;
    std::deque<OrePosition> orePositionHistory;

    OreAlignmentMessage solveDirection(OreAlignmentMessage& oreMessage, const OreAlignmentSettings& settings) {
        double tempDistance;

        switch(oreMessage.detectMode) {
            case OreDetectorMode::GOLD_OVER: {
                detectOres(oreMessage, { settings.goldOreHsvLow, settings.goldOreHsvHigh }, settings.overGoldAreaRange);
                addToHistory(detectLightBar(oreMessage.frame.frame, oreRectArray, settings), oreMessage.lastMode,
                             settings);  // todo:fix this for the no need of the history strategy
                uint32_t lightOutFrames = 0;
                std::for_each(orePositionHistory.begin(), orePositionHistory.end(), [&lightOutFrames](OrePosition& temp) {
                    lightOutFrames += (temp.flashingIndex == temp.totalNum) ? 0 : 1;
                });
                if(lightOutFrames / settings.historyFrameCount >= settings.flashFrequencyLimit) {
                    std::for_each(orePositionHistory.rbegin(), orePositionHistory.rend(), [&](const OrePosition& atom) {
                        if(atom.flashingIndex != atom.totalNum)
                            tempDistance = transformToRealDistance(oreRectArray[atom.flashingIndex], oreMessage.frame, settings);
                    });
                } else {
                    tempDistance = NAN;
                }
            } break;

            case OreDetectorMode::GOLD_GROUND: {
                detectOres(oreMessage, { settings.goldOreHsvLow, settings.goldOreHsvHigh }, settings.groundGoldAreaRange);
                tempDistance = transformToRealDistance(oreRectArray[0], oreMessage.frame, settings);
                for(auto& atom : oreRectArray) {
                    tempDistance = (transformToRealDistance(atom, oreMessage.frame, settings) < abs(tempDistance)) ?
                        transformToRealDistance(atom, oreMessage.frame, settings) :
                        tempDistance;
                }
            } break;

            case OreDetectorMode::SILVER_GROUND: {
                detectOres(oreMessage, { settings.silverOreHsvLow, settings.silverOreHsvHigh }, settings.groundSilverAreaRange);
                tempDistance = transformToRealDistance(oreRectArray[0], oreMessage.frame, settings);
                for(auto& atom : oreRectArray) {
                    tempDistance = (transformToRealDistance(atom, oreMessage.frame, settings) < abs(tempDistance)) ?
                        transformToRealDistance(atom, oreMessage.frame, settings) :
                        tempDistance;
                }
            } break;

            default:
                tempDistance = NAN;
                break;
        }
        return OreAlignmentMessage{ oreMessage.frame, OreDetectorMode::NONE, oreMessage.detectMode, tempDistance };
    }

    void detectOres(OreAlignmentMessage& message, const std::vector<std::vector<double>>& hsvRange,
                    const std::vector<uint32_t>& areaRange) {
        std::vector<std::vector<cv::Point2i>> orePointsArray;
        switch(message.detectMode) {
            case OreDetectorMode::SILVER_GROUND:
                inRange(hsvFrame, hsvRange[0], hsvRange[1], binaryOreFrame);
                break;
            default:
                inRange(hsvFrame, hsvRange[0], hsvRange[1], binaryOreFrame);
                break;
        }
        // cv::Mat tempMat = binaryOreFrame.clone();
        // dilate(tempMat, binaryOreFrame, std::vector<int32_t>{ 1, 1, 1, 1, 1, 1, 1, 1, 1 });  // todo:improve ore's shape

        findContours(binaryOreFrame, orePointsArray, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for(const auto& singleMine : orePointsArray) {

            cv::Rect_<int64_t> tempRect = boundingRect(singleMine);
            if(areaRange[0] < tempRect.area() && areaRange[1] > tempRect.area()) {
                oreRectArray.push_back(tempRect);
                cv::rectangle(bgrFrame, tempRect, cv::Scalar(255, 0, 0), 1, cv::LINE_8);
            }
        }
        std::sort(oreRectArray.begin(), oreRectArray.end(),
                  [](const cv::Rect_<int64_t>& front, const cv::Rect_<int64_t>& back) { return front.x < back.x; });
    }

    OrePosition detectLightBar(cv::Mat& frame, const std::vector<cv::Rect_<int64_t>>& oreRectArray,
                               const OreAlignmentSettings& settings) {
        cv::Mat lightBarFrame;
        uint64_t index = 0;
        std::vector<std::vector<cv::Point2i>> lightBarPointsArray;
        for(const auto& oreRect : oreRectArray) {
            cv::Mat cutFrame = frame.rowRange(oreRect.y - settings.heightExpand - settings.altitude,
                                              oreRect.y + oreRect.height + settings.heightExpand - settings.altitude);
            cutFrame = cutFrame.colRange(oreRect.x - settings.widthExpand, oreRect.x + oreRect.width + settings.widthExpand);

            cv::inRange(cutFrame, settings.lightBarHsvLow, settings.lightBarHsvHigh, lightBarFrame);
            cv::findContours(lightBarFrame, lightBarPointsArray, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for(const auto& singleLightBar : lightBarPointsArray) {
                cv::Rect tempRect = cv::boundingRect(singleLightBar);
                if(settings.lightbarAreaRange[0] < tempRect.area() && tempRect.area() < settings.lightbarAreaRange[1]) {
                    ++index;
                    // draw the signal light on frame, could be removed
                    tempRect = cv::Rect(cv::Point2i(tempRect.x + oreRect.x - settings.widthExpand,
                                                    tempRect.y + oreRect.y - settings.heightExpand),
                                        tempRect.size());
                    cv::rectangle(bgrFrame, tempRect, cv::Scalar(0, 255, 0), 1, cv::LINE_8);
                }
            }
        }
        return OrePosition{ oreRectArray.size(), index };
    }

    void addToHistory(const OrePosition& currentPosition, OreDetectorMode lastMode, const OreAlignmentSettings& settings) {
        if(lastMode != OreDetectorMode::GOLD_OVER && !orePositionHistory.empty()) {
            orePositionHistory.clear();
        } else {
            if(orePositionHistory.size() < settings.historyFrameCount)
                orePositionHistory.push_back(currentPosition);
            else {
                orePositionHistory.pop_front();
                orePositionHistory.push_back(currentPosition);
            }
        }  // todo:add frame filter
    }

    double transformToRealDistance(const cv::Rect_<int64_t>& rect, const CameraFrame& frame,
                                   const OreAlignmentSettings& settings) {
        return (rect.x + rect.width / 2 - frame.info.width) / (frame.info.width / 2 / tan(glm::radians(frame.info.fov))) *
            settings.distanceToOre +
            settings.offset;
    }  // todo:may don't have enough precision!!!

public:
    OreAlignment(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(OreAlignment).hash_code() } {}

    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](ore_alignment_available_atom, Identifier key) {
                     const auto settings = BlackBoard::instance().get<OreAlignmentSettings>(key).value();
                     auto data = BlackBoard::instance().get<OreAlignmentMessage>(key).value();

                     data.frame.frame.convertTo(bgrFrame, CV_32FC3, 1.0 / 255.0);
                     cv::cvtColor(bgrFrame, hsvFrame, cv::COLOR_BGR2HSV_FULL);

                     auto res = solveDirection(data, settings);
                     BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(ore_alignment_available_atom_v, mKey);
                 } };
    }
};

HUB_REGISTER_CLASS(OreAlignment);
