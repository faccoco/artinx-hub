#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedTarget.hpp"
#include "EnergyDetect.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <glm/glm.hpp>

struct HeroStrategySettings final {};

template <class Inspector>
bool inspect(Inspector& f, HeroStrategySettings& x) {
    return f.object(x).fields();
}

class HeroStrategy final : public HubHelper<caf::event_based_actor, HeroStrategySettings, set_target_atom> {
    Identifier mKey;

public:
    HeroStrategy(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(HeroStrategy).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](detect_available_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(detect_available_atom, TypedIdentifier<DetectedTargetArray>);
                     const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                     SelectedTarget selected;
                     selected.lastUpdate = data.lastUpdate;

                     auto minDistance = std::numeric_limits<double>::max();
                     for(auto& target : data.targets) {
                         const auto distance = glm::length(target.center.raw());
                         if(distance < minDistance) {
                             selected.selected = target;
                             minDistance = distance;
                         }
                     }
                     if(!selected.selected.has_value())
                         return;
                     sendAll(set_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
                 } };
    }
};

HUB_REGISTER_CLASS(HeroStrategy);
