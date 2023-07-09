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

struct BotsLocatorSettings final {
    std::string mapPath;
};

template <class Inspector>
bool inspect(Inspector& f, BotsLocatorSettings& x) {
    return f.object(x).fields(f.field("mapPath", x.mapPath));
}

class BotsLocator final : public HubHelper<caf::event_based_actor, BotsLocatorSettings, image_frame_atom, sync_position_atom> {
private:
    Identifier mKey;
    const cv::Mat mMap;
    bool mStart = false;
    std::array<bool, 12> redUsed, blueUsed;

    void debugView(const std::string_view& name, const cv::Mat& src, const std::function<void(cv::Mat&)>& func) {

        const auto hash = std::hash<std::string_view>{}(name);
        const Identifier newKey{ mKey.val ^ hash };

        cv::Mat res;
        src.copyTo(res);
        if(res.depth() == CV_8U)
            cv::putText(res, name.data(), { 0, 20 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{ 255 });
        else
            cv::putText(res, name.data(), { 0, 20 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{ 0, 255, 0 });

        func(res);

        CameraFrame frame;
        frame.frame = std::move(res);

        sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(newKey, std::move(frame), name));
    }

    static Color getCarColor(const cv::Mat& frame, const yolo::Box& box) {
        return Color::Red;
        cv::Mat subFrame(frame(cv::Rect(box.left, box.top, box.right - box.left, box.bottom - box.top)));
        size_t redCount = 0, blueCount = 0;
        for(int i = 0; i < subFrame.rows; ++i) {
            auto ptr = subFrame.ptr(i);
            for(int j = 0; j < subFrame.cols; ++j) {
                if(static_cast<int>(ptr[0] - ptr[2]) > 100)
                    ++blueCount;
                else if(static_cast<int>(ptr[2] - ptr[0]) > 100)
                    ++redCount;
            }
        }
        return redCount > blueCount ? Color::Red : Color::Blue;
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
                cv::putText(resultMap, std::to_string(bot.id), { static_cast<int>(bot.x * 100), static_cast<int>(bot.y * 100) },
                            cv::FONT_HERSHEY_SIMPLEX, 5, { 0, 255, 0 });
            }
        }
        debugView("ResultView", resultMap, [](auto) {});
    }

    uint16_t generateId(const Color carColor) {
        int index = 0;
        if(carColor == Color::Blue) {
            for(; index < blueUsed.size(); ++index)
                if(!blueUsed[index]) {
                    blueUsed[index] = true;
                    break;
                }
            return index + 102;
        } else if(carColor == Color::Red) {
            for(; index < redUsed.size(); ++index)
                if(!redUsed[index]) {
                    redUsed[index] = true;
                    break;
                }
            redUsed[index] = true;
            return index + 1;
        }
        return 0;
    }

    BotsPosition locate(const DetectedBots& botsInfo) {
        if(!mStart)
            return {};
        BotsPosition res;
        std::vector<cv::Point2f> botCenters;
        std::vector<cv::Point2f> dstResult;
        std::vector<Color> colorInfo;

        for(const auto& bot : botsInfo.botBoxes) {
            botCenters.emplace_back((bot.right + bot.left) / 2, bot.bottom);
            colorInfo.push_back(getCarColor(botsInfo.frame.frame, bot));
        }

        cv::perspectiveTransform(botCenters, dstResult, RadarPerspectiveTransform::instant().load());

        std::fill_n(redUsed.begin(), 12, false);
        std::fill_n(blueUsed.begin(), 12, false);
        for(int i = 0; i < dstResult.size(); ++i) {
            uint16_t tmpId = generateId(colorInfo[i]);
            res.data.push_back({ tmpId, std::abs(dstResult[i].x / 100), std::abs(dstResult[i].y / 100) });
        }
        return res;
    }

public:
    BotsLocator(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ generateKey(this) }, mMap(cv::imread(mConfig.mapPath)) {}

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [this](bots_locate_request_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(bots_locate_request_atom, TypedIdentifier<DetectedBots>);
                     if(const auto data = BlackBoard::instance().get<DetectedBots>(key)) {
                         if(!mStart)
                             mStart = RadarPerspectiveTransform::instant().isReady();

                         showPerspectiveResult(data.value().frame.frame);
                         auto res = locate(data.value());
                         showLocateResult(res);
                         HubLogger::watch("Count", res.data.size());
                         // logInfo("-----------------------------------");
                         // for(const auto& bot : res.data) {
                         // logInfo(fmt::format("Bot index: {}, x: {} y: {}", bot.id, bot.x, bot.y));
                         //}
                         sendAll(sync_position_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(BotsLocator);
#endif
