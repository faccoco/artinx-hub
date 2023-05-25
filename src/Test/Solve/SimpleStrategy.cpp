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

struct SimpleStrategySettings final {
    double staticImgPosThreshold;
    double maxMatchImgDistance;
    std::vector<int> priorList;
};

template <class Inspector>
bool inspect(Inspector& f, SimpleStrategySettings& x) {
    return f.object(x).fields(f.field("staticImgPosThreshold", x.staticImgPosThreshold).fallback(1),
                              f.field("maxMatchImgDistance", x.maxMatchImgDistance).fallback(10),
                              f.field("priorList", x.priorList)
                                  .invariant([](auto& ids) {
                                      for(auto id : ids) {
                                          if(id >= magic_enum::enum_count<RobotType>() || id < 0)
                                              return false;
                                      }
                                      return true;
                                  })
                                  .fallback(std::vector<int>()));
}

class SimpleStrategy final : public HubHelper<caf::event_based_actor, SimpleStrategySettings, set_target_atom,
                                              set_period_target_atom, set_period_outpost_atom> {
    Identifier mKey;

    constexpr static Duration mTrackValidTime = 50ms;
    constexpr static int mTrackSize = 10;

    std::list<std::queue<std::pair<TimePoint, cv::Point2f>>> mTrackedArmors;

    bool mPeriodActive = false, mPeriodToStart = false, mPeriodToInit = false, mPriorActive = false;
    int mPrior[magic_enum::enum_count<RobotType>()];

public:
    SimpleStrategy(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        memset(mPrior, 0, magic_enum::enum_count<RobotType>() * sizeof(int));
        int prior = mConfig.priorList.size() + 1;
        for(auto id : mConfig.priorList)
            mPrior[id] = (--prior);
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
                // process whether armor is static
                for(auto& target : selected.targets) {
                    bool matched = false;
                    for(auto& tracked : mTrackedArmors) {
                        // matched
                        if(distance2D(tracked.back().second, target.armorImgCenter) < mConfig.maxMatchImgDistance) {
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
                }

                if(mPeriodActive) {
                    if(mPriorActive) {
                        sendAll(set_period_outpost_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected),
                                mPeriodToInit);
                    } else {
                        sendAll(set_period_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected),
                                mPeriodToInit);
                    }
                    mPeriodToInit = false;
                } else {
                    if(!selected.targets.empty()) {
                        std::sort(selected.targets.begin(), selected.targets.end(),
                                  [this](const DetectedTarget& lhs, const DetectedTarget& rhs) {
                                      if(mPrior[lhs.id] != mPrior[rhs.id])
                                          return mPrior[lhs.id] > mPrior[rhs.id];
                                      if(lhs.id == RobotType::Outpost)
                                          if(lhs.motion != rhs.motion)
                                              return lhs.motion == ArmorMotion::Static;
                                      return lhs.distToImgCenter < rhs.distToImgCenter;
                                  });
                        selected.selected = selected.targets[0];
                        sendAll(set_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
                    }
                }
            },
            [this](hero_strategy_control_atom, bool periodActive, bool priorActive) {
                ACTOR_PROTOCOL_CHECK(hero_strategy_control_atom, bool, bool);
                if((periodActive && !mPeriodActive) || (priorActive ^ mPriorActive)) {
                    mPeriodToStart = true;
                    mPeriodToInit = true;
                }
                mPeriodActive = periodActive;
                mPriorActive = priorActive;
            },
        };
    }
};

HUB_REGISTER_CLASS(SimpleStrategy);
