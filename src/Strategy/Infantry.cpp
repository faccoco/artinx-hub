#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedTarget.hpp"
#include "EnergyDetect.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <glm/glm.hpp>

struct InfantryStrategySettings final {};

template <class Inspector>
bool inspect(Inspector& f, InfantryStrategySettings& x) {
    return f.object(x).fields();
}

class InfantryStrategy final : public HubHelper<caf::event_based_actor, InfantryStrategySettings, set_target_atom> {
    Identifier mKey;
    bool mEnergyMode = true;  // test only

public:
    InfantryStrategy(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(InfantryStrategy).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [](start_atom) {}, [&](energy_detector_control_atom, bool enable) { mEnergyMode = enable; },
                 [&](energy_detect_available_atom, Identifier key) {
                     if(!mEnergyMode)
                         return;
                     const auto data = BlackBoard::instance().get<DetectedEnergyInfo>(key).value();
                     SelectedTarget selected;
                     selected.lastUpdate = data.lastUpdate;
                     selected.center = data.point;
                     BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected);
                     sendAll(set_target_atom_v, mKey);
                 },
                 [&](detect_available_atom, Identifier key) {
                     if(mEnergyMode)
                         return;
                     const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                     SelectedTarget selected;
                     selected.lastUpdate = data.lastUpdate;

                     auto minDistance = std::numeric_limits<double>::max();
                     for(auto& target : data.targets) {
                         const auto distance = glm::length(target.center.raw());
                         if(distance < minDistance) {
                             selected.center = target.center;
                             minDistance = distance;
                         }
                     }
                     if(!selected.center.has_value())
                         return;
                     BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected);
                     sendAll(set_target_atom_v, mKey);
                 } };
    }
};

HUB_REGISTER_CLASS(InfantryStrategy);
