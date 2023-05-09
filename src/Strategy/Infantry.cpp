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

struct InfantryStrategySettings final {};

template <class Inspector>
bool inspect(Inspector& f, InfantryStrategySettings& x) {
    return f.object(x).fields();
}

class InfantryStrategy final
    : public HubHelper<caf::event_based_actor, InfantryStrategySettings, set_target_atom, update_roi_atom> {
    Identifier mKey;
    bool mEnergyMode = false;

public:
    InfantryStrategy(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](energy_detector_control_atom, bool enable) {
                     ACTOR_PROTOCOL_CHECK(energy_detector_control_atom, bool);
                     mEnergyMode = enable;
                 },
                 [&](energy_detect_available_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(energy_detect_available_atom, TypedIdentifier<DetectedEnergyInfo>);
                     if(!mEnergyMode)
                         return;
                     const auto data = BlackBoard::instance().get<DetectedEnergyInfo>(key).value();
                     SelectedTarget selected;
                     selected.lastUpdate = data.lastUpdate;

                     sendAll(set_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
                 },
                 [&](detect_available_atom, GroupMask, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
                     if(mEnergyMode)
                         return;
                     const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                     SelectedTarget selected;
                     selected.lastUpdate = data.lastUpdate;
                     selected.tfRobot2Gun = data.tfRobot2Gun;

                     std::optional<DetectedTarget> minDistTarget;
                     auto minDistance = 10000.0;
                     for(auto& target : data.targets) {
                         auto pos = target.center.mVal;
                         auto dist = pos.x * pos.x + pos.y * pos.y;
                         if(dist < minDistance) {
                             minDistance = dist;
                             minDistTarget = target;
                         }
                     }

                     selected.selected = minDistTarget;

                     sendAll(set_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
                 } };
    }
};

HUB_REGISTER_CLASS(InfantryStrategy);
