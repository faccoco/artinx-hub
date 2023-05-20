#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedTarget.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include "Utility.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"

struct HeroStrategySettings final {
    std::string periodPredictType;
    double staticImgPosThreshold;
    double maxMatchImgDistance;
};

template <class Inspector>
bool inspect(Inspector& f, HeroStrategySettings& x) {
    return f.object(x).fields(f.field("periodPredictType", x.periodPredictType).fallback("outpost"),
                              f.field("staticImgPosThreshold", x.staticImgPosThreshold).fallback(1),
                              f.field("maxMatchImgDistance", x.maxMatchImgDistance).fallback(10));
}

class HeroStrategy final : public HubHelper<caf::event_based_actor, HeroStrategySettings, set_target_atom, set_period_target_atom,
                                            set_period_outpost_atom> {
    Identifier mKey;

    constexpr static Duration mTrackValidTime = 50ms;
    constexpr static int mTrackSize = 10;

    std::function<void(SelectedTarget)> mSendPeriodFunc;
    bool mPeriodActive = false, mPeriodInited = false;

    std::list<std::queue<std::pair<TimePoint, cv::Point2f>>> mTrackedArmors;

public:
    HeroStrategy(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        if(mConfig.periodPredictType == "outpost")
            mSendPeriodFunc = [this](const SelectedTarget& selected) {
                sendAll(set_period_outpost_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected),
                        !mPeriodInited);
            };
        else if(mConfig.periodPredictType == "car")
            mSendPeriodFunc = [this](const SelectedTarget& selected) {
                sendAll(set_period_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected),
                        !mPeriodInited);
            };
        else {
            logInfo("HeroStrategy wrong periodPredictType using fallback \"outpost\"");
            mSendPeriodFunc = [this](const SelectedTarget& selected) {
                sendAll(set_period_outpost_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected),
                        !mPeriodInited);
            };
        }
    }
    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](detect_available_atom, GroupMask, Identifier key) {
                ACTOR_PROTOCOL_CHECK(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
                const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                SelectedTarget selected;
                selected.lastUpdate = data.lastUpdate;
                selected.tfRobot2Gun = data.tfRobot2Gun;
                selected.targets = data.targets;

                // exclude invalid data in mTrackedArmors
                for(auto trackedIter = mTrackedArmors.begin(); trackedIter != mTrackedArmors.end(); trackedIter++) {
                    while(!trackedIter->empty() && trackedIter->front().first - data.lastUpdate > mTrackValidTime)
                        trackedIter->pop();
                    if(trackedIter->empty())
                        mTrackedArmors.erase(trackedIter);
                }

                // std::string tmp = "";
                // process whether armor is static
                for(auto& target : selected.targets) {
                    bool matched = false;
                    for(auto& tracked : mTrackedArmors) {
                        // matched
                        if(distance2D(tracked.back().second, target.armorImgCenter) < mConfig.maxMatchImgDistance) {
                            //                            logInfo(fmt::format("{}", distance2D(tracked.front().second,
                            //                            target.armorImgCenter)));
                            if(distance2D(tracked.front().second, target.armorImgCenter) < mConfig.staticImgPosThreshold)
                                target.motion = ArmorMotion::Static;
                            else
                                target.motion = ArmorMotion::Moving;
                            matched = true;
                            if(tracked.size() == mTrackSize)
                                tracked.pop();
                            tracked.emplace(data.lastUpdate, target.armorImgCenter);
                            break;
                        }
                    }
                    // not find matched armor so it's a new armor
                    if(!matched) {
                        mTrackedArmors.emplace_back();
                        mTrackedArmors.back().emplace(data.lastUpdate, target.armorImgCenter);
                        target.motion = ArmorMotion::Unsure;
                        continue;
                    }
                    // tmp += fmt::format("{} ", magic_enum::enum_name(target.motion));
                    //                    logInfo(magic_enum::enum_name(target.motion));
                }
                // logInfo(tmp);

                HubLogger::VisualLog(fmt::format("Hero: mPeriodActive:{}", mPeriodActive));

                if(mPeriodActive) {
                    mSendPeriodFunc(selected);
                    mPeriodInited = true;
                } else {
                    double minDisToImgCenter = std::numeric_limits<double>::max();
                    for(const auto& target : selected.targets) {
                        if(target.distToImgCenter < minDisToImgCenter) {
                            selected.selected = target;
                            minDisToImgCenter = target.distToImgCenter;
                        }
                    }
                    sendAll(set_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
                }
            },
            [this](outpost_detector_control_atom, bool active) {
                ACTOR_PROTOCOL_CHECK(outpost_detector_control_atom, bool);
                if(active && !mPeriodActive)
                    mPeriodInited = false;
                mPeriodActive = active;
            },
        };
    }
};

HUB_REGISTER_CLASS(HeroStrategy);
