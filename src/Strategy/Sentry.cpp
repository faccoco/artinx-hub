#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedTarget.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/glm.hpp>

#include "SuppressWarningEnd.hpp"

struct SentryStrategySettings final {
    double distanceThreshold;
    double detectedTTL;
};

constexpr int32_t engineerId = 2;

template <class Inspector>
bool inspect(Inspector& f, SentryStrategySettings& x) {
    return f.object(x).fields(f.field("distanceThreshold", x.distanceThreshold), f.field("detectedTTL", x.detectedTTL));
}

class SentryStrategy final : public HubHelper<caf::event_based_actor, SentryStrategySettings, set_target_atom> {
    Identifier mKey;
    SelectedTarget mLastSelected1, mLastSelected2;
//    Identifier mHead1, mHead2;

public:
    SentryStrategy(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [&](detect_available_atom, GroupMask mask, Identifier key) {
                ACTOR_PROTOCOL_CHECK(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
                const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                SelectedTarget selTarget;
                selTarget.lastUpdate = data.lastUpdate;
                selTarget.tfRobot2Gun = data.tfRobot2Gun;

                // Give priority to striking the nearest large armor plate target
                auto minDistance = mConfig.distanceThreshold;
                auto curArmorType = ArmorType::Small;
                for(auto& target : data.targets) {  // 遍历检测到的目标装甲板信息
                    const auto distance = glm::length(target.center.mVal);
                    if(target.type == ArmorType::Large) {  // 如果当前遍历的装甲板为大装甲板
                        if(curArmorType == ArmorType::Small ||
                           distance < minDistance) {  // 之前遍历的装甲板为小装甲板或者距离比上一次遍历的装甲板距离近
                            selTarget.selected = target;  // 选择当前遍历的装甲板为目标装甲板
                            minDistance = distance;
                            curArmorType = ArmorType::Large;
                        }
                    } else {  // 否则，当前遍历的为小装甲板
                        if(curArmorType == ArmorType::Small && distance < minDistance &&
                           target.id !=
                               engineerId) {  // 如果之前遍历的装甲板都是小装甲板,并且该次遍历的装甲板距离近，并且不是工程机器人的装甲板
                            selTarget.selected = target;
                            minDistance = distance;
                        }
                    }
                }

                if(selTarget.selected.has_value()) {
                    (mask == 1U ? mLastSelected1 : mLastSelected2) = selTarget;
                } else {
                    selTarget = (mask == 1U) ? mLastSelected1 : mLastSelected2;
                    const auto& delta = Clock::now() - selTarget.lastUpdate;
                    if(!selTarget.selected.has_value() || !(delta.count() < static_cast<Clock::rep>(mConfig.detectedTTL * 1e9))) {
                        return;
                    }
                }

                sendMasked(set_target_atom_v, mask,
                           BlackBoard::instance().updateSync<SelectedTarget>(Identifier{ mKey.val ^ mask }, selTarget));
            }
        };
    }
};

HUB_REGISTER_CLASS(SentryStrategy);
