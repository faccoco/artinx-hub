#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedTarget.hpp"
#include "EnergyDetect.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <glm/glm.hpp>

#include "SuppressWarningEnd.hpp"

struct SentryStrategySettings final {
    std::vector<int> ignoredId;
    float maxDistance;
};

template <class Inspector>
bool inspect(Inspector& f, SentryStrategySettings& x) {
    return f.object(x).fields(f.field("ignoredId", x.ignoredId), f.field("maxDistance", x.maxDistance).fallback(8.0));
}

// 如果上一次有选择目标，则选择与上次目标id相同的装甲板进行击打；
// 否则，先选择英雄击打；
// 无英雄，无上次选择的相同目标，则击打最近的装甲板；
class SentryStrategy final : public HubHelper<caf::event_based_actor, SentryStrategySettings, set_target_atom> {
    Identifier mKey;
    SelectedTarget mLastTarget;
    std::set<int> mIgnoreId;

public:
    SentryStrategy(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        for(auto id : mConfig.ignoredId) {
            mIgnoreId.insert(id);
        }
    }
    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](detect_available_atom, GroupMask, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
                     const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                     SelectedTarget selected;
                     selected.lastUpdate = data.lastUpdate;
                     selected.tfRobot2Gun = data.tfRobot2Gun;

                     std::optional<DetectedTarget> heroTarget, sameTarget, minDistTarget;
                     auto minDistance = 10000.0;
                     for(auto& target : data.targets) {
                         if(mIgnoreId.count(static_cast<int>(target.id)))
                             continue;
                         selected.targets.emplace_back(target);
                         auto pos = target.center.mVal;
                         auto dist = pos.x * pos.x + pos.y * pos.y + pos.z * pos.z;
                         if(dist > mConfig.maxDistance * mConfig.maxDistance)
                             continue;
                         if(dist < minDistance) {
                             minDistance = dist;
                             minDistTarget = target;
                         }
                         if(mLastTarget.selected.has_value() && mLastTarget.selected->id == target.id) {
                             sameTarget = target;
                         }
                         if(target.id == RobotType::Hero) {
                             heroTarget = target;
                         }
                     }

                     if(sameTarget.has_value()) {
                         selected.selected = sameTarget;
                     } else if(heroTarget.has_value()) {
                         selected.selected = heroTarget;
                     } else {
                         selected.selected = minDistTarget;
                     }
                     mLastTarget = selected;

                     if (selected.selected.has_value()){
                         HubLogger::VisualLog(fmt::format("SentryStrategy Receive {} targets, choose target: {}", selected.targets.size(), magic_enum::enum_name(selected.selected->id)));
                     }
                     sendAll(set_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
                 } };
    }
};

HUB_REGISTER_CLASS(SentryStrategy);