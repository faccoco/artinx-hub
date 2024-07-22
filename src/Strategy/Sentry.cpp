#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedTarget.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <cmath>
#include <glm/glm.hpp>
#include <vector>

#include "SuppressWarningEnd.hpp"

constexpr auto recordTime = 0.3s; // NOLINT

struct SentryStrategySettings final {
    std::vector<int> ignoredId;
    float maxDistance;
};

template <class Inspector>
bool inspect(Inspector& f, SentryStrategySettings& x) {
    return f.object(x).fields(f.field("ignoredId", x.ignoredId), f.field("maxDistance", x.maxDistance).fallback(8.0));
}

class SentryStrategy final : public HubHelper<caf::event_based_actor, SentryStrategySettings, set_target_atom> {
    Identifier mKey;
    SelectedTarget mLastTarget1, mLastTarget2;
    std::set<RobotType> mIgnoreId;

public:
    SentryStrategy(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, std::move(name) }, mKey{ generateKey(this) } {
        for(auto id : mConfig.ignoredId) {
            mIgnoreId.insert(static_cast<RobotType>(id));
        }
    }
    caf::behavior make_behavior() override {
        return {
            [](start_atom /*unused*/) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [&](detect_available_atom /*unused*/, GroupMask mask, Identifier key) {
                ACTOR_PROTOCOL_CHECK(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
                const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                std::set<RobotType> ignoreId;
                for(auto id : mConfig.ignoredId) {
                    ignoreId.insert(static_cast<RobotType>(id));
                }
                if (GlobalSettings::get().blockEngineer) {
                    ignoreId.insert(RobotType::Engineer);
                }
                if (GlobalSettings::get().blockSentry) {
                    ignoreId.insert(RobotType::Sentry);
                }


                // 先杀英雄，若无英雄选择最近目标
                SelectedTarget selected;
                selected.lastUpdate = data.lastUpdate;
                selected.tfRobot2Camera = data.tfRobot2Camera;

                auto minDistance = 10000.0;
                auto prior = GlobalSettings::get().priorNum;
                bool priorInit = false;
                std::vector<DetectedTarget> candTargets;
                for(auto& target : data.targets) {

                    if (target.id == static_cast<RobotType>(prior)) {
                        selected.targets.push_back(target);
                        priorInit = true;
                        break;
                    }

                    if (ignoreId.find(target.id) != ignoreId.end()) {
                        continue;
                    }

                    auto pos = target.center.mVal;
                    auto dist = std::sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
                    if (dist < minDistance) {
                        selected.selected = target;
                    }

                    if (dist > mConfig.maxDistance || pos.y > 1.5) {
                        continue;
                    }

                    candTargets.push_back(target);
                }
                if (!priorInit){
                    std::copy(candTargets.begin(), candTargets.end(), std::back_inserter(selected.targets));
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