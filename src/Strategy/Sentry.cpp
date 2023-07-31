#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedTarget.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <glm/glm.hpp>

#include "SuppressWarningEnd.hpp"

constexpr auto recordTime = 0.3s;

struct SentryStrategySettings final {
    std::vector<int> ignoredId;
    float maxDistance;
};

template <class Inspector>
bool inspect(Inspector& f, SentryStrategySettings& x) {
    return f.object(x).fields(f.field("ignoredId", x.ignoredId), f.field("maxDistance", x.maxDistance).fallback(8.0));
}

// 先杀英雄，若无英雄，则选择与上次目标id相同的装甲板进行击打；
// 无英雄，无上次选择的相同目标，则击打最近的装甲板；
class SentryStrategy final : public HubHelper<caf::event_based_actor, SentryStrategySettings, set_target_atom> {
    Identifier mKey;
    SelectedTarget mLastTarget1, mLastTarget2;
    std::set<int> mIgnoreId;

public:
    SentryStrategy(caf::actor_config& base, const HubConfig& config, std::string name) : HubHelper{ base, config, name }, mKey{ generateKey(this) } {
        for(auto id : mConfig.ignoredId) {
            mIgnoreId.insert(id);
        }
    }
    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [&](detect_available_atom, GroupMask mask, Identifier key) {
                ACTOR_PROTOCOL_CHECK(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
                const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                SelectedTarget selected;
                selected.lastUpdate = data.lastUpdate;
                selected.tfRobot2Camera = data.tfRobot2Camera;

                std::optional<DetectedTarget> heroTarget, sameTarget, minDistTarget;
                auto minDistance = 10000.0;
                for(auto& target : data.targets) {
                    if(mIgnoreId.count(static_cast<int>(target.id)))
                        continue;
                    selected.targets.emplace_back(target);
                    auto pos = target.center.mVal;
                    auto dist = pos.x * pos.x + pos.y * pos.y + pos.z * pos.z;
                    if(dist > mConfig.maxDistance * mConfig.maxDistance || pos.y > 1.5)  // 距离太远或者高度超过1.5m不打弹
                        continue;
                    if(dist < minDistance) {
                        minDistance = dist;
                        minDistTarget = target;
                    }
                    if((mLastTarget1.selected.has_value() && mLastTarget1.selected->id == target.id) ||
                       (mLastTarget2.selected.has_value() && mLastTarget2.selected->id == target.id)) {
                        sameTarget = target;
                    }
                    if(target.id == RobotType::Hero) {
                        heroTarget = target;
                    }
                }

                if(heroTarget.has_value()) {
                    selected.selected = heroTarget;
                } else if(sameTarget.has_value()) {
                    selected.selected = sameTarget;
                } else {
                    selected.selected = minDistTarget;
                }

                if(mask == 1U) {
                    if(selected.selected.has_value()) {
                        mLastTarget1 = selected;
                        if(mLastTarget2.selected.has_value() && Clock::now() - mLastTarget2.lastUpdate < recordTime &&
                           mLastTarget2.selected->id == RobotType::Hero) {
                            return;
                        }
                    } else {
                        if(mLastTarget2.selected.has_value() && Clock::now() - mLastTarget2.lastUpdate < recordTime) {
                            return;
                        }
                    }
                } else {
                    if(selected.selected.has_value()) {
                        mLastTarget2 = selected;
                        if(mLastTarget1.selected.has_value() && Clock::now() - mLastTarget1.lastUpdate < recordTime &&
                           mLastTarget2.selected->id != RobotType::Hero) {
                            return;
                        }
                    } else {
                        if(mLastTarget1.selected.has_value() && Clock::now() - mLastTarget1.lastUpdate < recordTime) {
                            return;
                        }
                    }
                }

                if(selected.selected.has_value()) {
                    HubLogger::visualLog(fmt::format("SentryStrategy Receive {} targets, choose target: {}",
                                                     selected.targets.size(), magic_enum::enum_name(selected.selected->id)));
                }
                sendAll(set_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
            },
        };
    }
};

HUB_REGISTER_CLASS(SentryStrategy);