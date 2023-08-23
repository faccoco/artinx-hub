#ifdef ARTINX_RADAR
#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "RadarInfo.hpp"
#include "Utility.hpp"
#include "yolo.hpp"

#include "SuppressWarningBegin.hpp"

#include <algorithm>
#include <caf/event_based_actor.hpp>
#include <fmt/core.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <vector>

#include "SuppressWarningEnd.hpp"

struct BotLocatorSettings final {
    std::string mapPath;
    bool selfRed;
    double rivalRHeightOffset;
    double rivalRoundHeightOffset;
    double selfRHeightOffset;
    double selfRoundHeightOffset;
};

template <class Inspector>
bool inspect(Inspector& f, BotLocatorSettings& x) {
    return f.object(x).fields(
        f.field("mapPath", x.mapPath), f.field("selfRed", x.selfRed), f.field("rivalRHeightOffset", x.rivalRHeightOffset),
        f.field("rivalRoundHeightOffset", x.rivalRoundHeightOffset), f.field("selfRHeightOffset", x.selfRHeightOffset),
        f.field("selfRoundHeightOffset", x.selfRoundHeightOffset));
}

class BotLocator final : public HubHelper<caf::event_based_actor, BotLocatorSettings, image_frame_atom, sync_position_atom> {
private:
    Identifier mKey;
    const cv::Mat mMap;
    bool mStart = false;
    std::array<bool, 7> redUseage, blueUseage;

    void debugView(const std::string_view& name, const cv::Mat& src, const std::function<void(cv::Mat&)>& func) {

        const auto hash = std::hash<std::string_view>{}(name);
        const Identifier newKey{ mKey.val ^ hash };

        cv::Mat res;
        src.copyTo(res);
        if(res.depth() == CV_8U) {
            cv::putText(res, name.data(), { 0, 20 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{ 255 });
        } else {
            cv::putText(res, name.data(), { 0, 20 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{ 0, 255, 0 });
        }

        func(res);

        CameraFrame frame;
        frame.frame = std::move(res);

        sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(newKey, std::move(frame), name));
    }

    void showPerspectiveResult(const cv::Mat& frame) {
        cv::Mat perspectedView;
        if(mStart) {
            cv::warpPerspective(frame, perspectedView, RadarPerspectiveTransform::instant().load(), cv::Size{ 1500, 2800 });
        } else {
            perspectedView = mMap;
        }
        debugView("PerspectView", perspectedView, [](auto) {});
    }
    void showLocateResult(const BotsPosition& res) {
        cv::Mat resultMap = mMap.clone();
        if(mStart) {
            for(const auto& bot : res.data) {
                cv::putText(resultMap, std::to_string(bot.id),
                            { static_cast<int>(bot.x * 100), mMap.rows - static_cast<int>(bot.y * 100) },
                            cv::FONT_HERSHEY_SIMPLEX, 5, { 0, 255, 0 });
            }
        }
        debugView("ResultView", resultMap, [](auto) {});
    }

    std::vector<uint16_t> generateId(const std::vector<yolo::Box>& boxes) {
        std::vector<size_t> noNumber;
        std::vector<uint16_t> result(boxes.size());
        std::fill_n(redUseage.begin(), 7, false);
        std::fill_n(blueUseage.begin(), 7, false);
        for(size_t i = 0; i < boxes.size(); ++i) {
            if(boxes[i].class_label == 10 || boxes[i].class_label == 19) {
                noNumber.push_back(i);
                continue;
            } else if(boxes[i].class_label > 1 && boxes[i].class_label <= 9) {  // blue
                result[i] = boxes[i].class_label + 99;
                blueUseage[i] = true;
            } else if(boxes[i].class_label > 10 && boxes[i].class_label <= 18) {  // red
                result[i] = boxes[i].class_label - 10;
                redUseage[i] = true;
            }
        }
        for(int index : noNumber) {
            if(boxes[index].class_label == 10) {  // blue
                for(size_t i = 0; i < blueUseage.size(); ++i) {
                    if(!blueUseage[i]) {
                        blueUseage[i] = true;
                        //                        result[index] = 6 - i + 101;
                        result[index] = 6 + 101;
                        break;
                    }
                }
            } else {  // red
                for(size_t i = 0; i < redUseage.size(); ++i) {
                    if(!redUseage[i]) {
                        redUseage[i] = true;
                        //                        result[index] = 6 - i + 1;
                        result[index] = 6 + 1;
                        break;
                    }
                }
            }
        }
        return result;
    }

    void fixOffset(BotsPosition& origin) {
        constexpr static auto inRedRHeightRange = [](const DetectedBotPosition& bot) {
            return (bot.x > 3.4 && bot.x < 9.5) && (/*r3*/ bot.y > 10.7 && bot.y < 15);
        };
        constexpr static auto inBlueRHeightRange = [](const DetectedBotPosition& bot) {
            return (bot.x > 18.5 && bot.x < 24.6) && (/*r3*/ bot.y > 9 && bot.y < 4.3);
        };
        constexpr static auto inRedRoundHeightRange = [](const DetectedBotPosition& bot) {
            return (bot.x > 9.75 && bot.x < 12.75) && (bot.y > 8 && bot.y < 11.5);
        };
        constexpr static auto inBlueRoundHeightRange = [](const DetectedBotPosition& bot) {
            return (bot.x > 15.25 && bot.x < 18.25) && (bot.y > 3.5 && bot.y < 7);
        };
        if(mConfig.selfRed) {
            for(auto& bot : origin.data) {
                if(inBlueRHeightRange(bot)) {
                    bot.x -= mConfig.rivalRHeightOffset;
                } else if(inBlueRoundHeightRange(bot)) {
                    bot.x -= mConfig.rivalRoundHeightOffset;
                }
                if(inRedRHeightRange(bot)) {
                    bot.x -= mConfig.selfRHeightOffset;
                } else if(inRedRoundHeightRange(bot)) {
                    bot.x -= mConfig.selfRoundHeightOffset;
                }
            }
        } else {
            for(auto& bot : origin.data) {
                if(inRedRHeightRange(bot)) {
                    bot.x += mConfig.rivalRHeightOffset;
                } else if(inRedRoundHeightRange(bot)) {
                    bot.x += mConfig.rivalRoundHeightOffset;
                }
                if(inBlueRHeightRange(bot)) {
                    bot.x += mConfig.selfRHeightOffset;
                } else if(inBlueRoundHeightRange(bot)) {
                    bot.x += mConfig.selfRoundHeightOffset;
                }
            }
        }
    }

    BotsPosition locate(const DetectedBots& botsInfo) {
        if(!mStart)
            return {};
        if(botsInfo.botBoxes.empty())
            return {};
        BotsPosition result;
        std::vector<cv::Point2f> botCenters;
        std::vector<cv::Point2f> dstResult;
        for(const auto& bot : botsInfo.botBoxes) {
            botCenters.emplace_back((bot.right + bot.left) / 2, bot.bottom);
        }

        cv::perspectiveTransform(botCenters, dstResult, RadarPerspectiveTransform::instant().load());

        auto idArray = generateId(botsInfo.botBoxes);
        if(mConfig.selfRed) {
            for(size_t i = 0; i < dstResult.size(); ++i) {
                result.data.push_back({ idArray[i], 28 - std::abs(dstResult[i].y / 100), 15 - std::abs(dstResult[i].x / 100) });
            }
        } else {
            for(size_t i = 0; i < dstResult.size(); ++i) {
                result.data.push_back({ idArray[i], std::abs(dstResult[i].y / 100), std::abs(dstResult[i].x / 100) });
            }
        }
        return result;
    }

public:
    BotLocator(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, std::move(name) }, mKey{ generateKey(this) }, mMap(cv::imread(mConfig.mapPath)) {}

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [this](bot_locate_request_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(bot_locate_request_atom, TypedIdentifier<DetectedBots>);
                     if(const auto data = BlackBoard::instance().get<DetectedBots>(key)) {
                         mStart = RadarPerspectiveTransform::instant().isReady();

                         showPerspectiveResult(data.value().frame.frame);
                         auto res = locate(data.value());
                         fixOffset(res);
                         HubLogger::watch("Solve bots:", res.data.size());
                         HubLogger::visualLog("===== Finish pos solve =====");
                         for(size_t i = 0; i < res.data.size(); ++i) {
                             HubLogger::visualLog(
                                 fmt::format("ID: {} pos: [ x: {}, y: {} ]", res.data[i].id, res.data[i].x, res.data[i].y));
                         }
                         showLocateResult(res);
                         sendAll(sync_position_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(BotLocator);
#endif
