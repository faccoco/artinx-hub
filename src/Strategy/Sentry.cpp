#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedTarget.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <glm/glm.hpp>

struct SentryStrategySettings final {
    double distanceThreshold;
};

constexpr int32_t engineerId = 2;

template <class Inspector>
bool inspect(Inspector& f, SentryStrategySettings& x) {
    return f.object(x).fields(f.field("distanceThreshold", x.distanceThreshold));
}

class SentryStrategy final : public HubHelper<caf::event_based_actor, SentryStrategySettings, set_target_atom> {
    Identifier mKey;

public:
    SentryStrategy(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(SentryStrategy).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](detect_available_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(detect_available_atom, TypedIdentifier<DetectedTargetArray>);
                     const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                     SelectedTarget selected;
                     selected.lastUpdate = data.lastUpdate;

                     auto minDistance = std::numeric_limits<double>::max();
                     for(auto& target : data.targets) {
                         const auto distance = glm::length(target.center.raw());
                         if(distance > mConfig.distanceThreshold && distance < minDistance) {
                             if(target.id != engineerId) {
                                 selected.selected = target;
                                 minDistance = distance;
                             }
                         }
                     }
                     if(!selected.selected.has_value())
                         return;
                     sendAll(set_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
                 } };
    }
};

HUB_REGISTER_CLASS(SentryStrategy);
