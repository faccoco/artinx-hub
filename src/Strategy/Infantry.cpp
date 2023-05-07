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

struct InfantryStrategySettings final {

};

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
                 [&](detect_available_atom, GroupMask, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
                     const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                     SelectedTarget selected;
                     selected.lastUpdate = data.lastUpdate;
                     selected.tfRobot2Gun = data.tfRobot2Gun;
                     selected.targets = data.targets;

                     auto minDistance = std::numeric_limits<double>::max();
                     for(auto& target : data.targets) {
                         const auto vec = target.center.mVal;
                         const auto distance = vec.x * vec.x + vec.y * vec.y;
                         if(distance < minDistance) {
                             selected.selected = target;
                             minDistance = distance;
                         }
                     }

                     sendAll(set_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
                     if(selected.selected.has_value()) {
                         TargetROI roi{ selected.lastUpdate, minDistance, selected.selected->armorImgCenter };
                         sendAll(update_roi_atom_v, BlackBoard::instance().updateSync<TargetROI>(mKey, roi));
                     }

                     if(selected.selected.has_value())
                         HubLogger::watch("armor type", magic_enum::enum_name(selected.selected->type));
                 } };
    }
};

HUB_REGISTER_CLASS(InfantryStrategy);
