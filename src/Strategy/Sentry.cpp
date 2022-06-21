#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedTarget.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <glm/glm.hpp>

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
    Identifier mHead1, mHead2;

public:
    SentryStrategy(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](detect_available_atom, GroupMask mask, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
                     const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                     SelectedTarget selected;
                     selected.lastUpdate = data.lastUpdate;

                     auto minDistance = mConfig.distanceThreshold;
                     for(auto& target : data.targets) {
                         const auto distance = glm::length(target.center.raw());
                         if(distance < minDistance && target.id != engineerId) {
                             selected.selected = target;
                             minDistance = distance;
                         }
                     }

                     if(selected.selected.has_value()) {
                         (mask == 1U ? mLastSelected1 : mLastSelected2) = selected;
                     } else {
                         const auto head1 = BlackBoard::instance().get<HeadInfo>(mHead1);
                         const auto head2 = BlackBoard::instance().get<HeadInfo>(mHead2);
                         if(!(head1.has_value() && head2.has_value())) {
                             logWarning("No head info for sentry");
                             return;
                         }

                         const auto& headInfo1 = head1.value();
                         const auto& headInfo2 = head2.value();

                         selected = (mask == 1U ? mLastSelected2 : mLastSelected1);

                         if(!selected.selected.has_value())
                             return;

                         const auto delta = Clock::now() - selected.lastUpdate;
                         if(delta.count() > static_cast<Clock::rep>(mConfig.detectedTTL * 1e9))
                             return;

                         auto& center = selected.selected.value().center;
                         auto& velocity = selected.selected.value().velocity;
                         if(mask == 1U) {
                             center = headInfo1.transform(headInfo1.transform(center));
                             velocity = headInfo1.transform(headInfo1.transform(velocity));
                         } else {
                             center = headInfo2.transform(headInfo2.transform(center));
                             velocity = headInfo2.transform(headInfo2.transform(velocity));
                         }
                     }

                     sendMasked(set_target_atom_v, mask, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
                 },
                 [&](update_head_atom, GroupMask mask, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(update_head_atom, GroupMask, TypedIdentifier<HeadInfo>);
                     (mask == 1U ? mHead1 : mHead2) = key;
                 } };
    }
};

HUB_REGISTER_CLASS(SentryStrategy);
