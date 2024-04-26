#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedEnergyFan.hpp"
#include "DetectedTarget.hpp"
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

class InfantryStrategy final : public HubHelper<caf::event_based_actor, InfantryStrategySettings, set_target_atom> {
    Identifier mKey;

public:
    InfantryStrategy(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, std::move(name) }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](detect_available_atom, GroupMask, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
                     const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                     SelectedTarget selected;
                     selected.lastUpdate = data.lastUpdate;
                     selected.tfRobot2Camera = data.tfRobot2Camera;
                     AutoAimMode autoAimMode = GlobalSettings::get().getAutoAimMode();
                     RobotType PriorTarget;
                     switch(autoAimMode) {
                         case AutoAimMode::HeroFirst:
                             PriorTarget = RobotType::Hero;
                             break;
                         case AutoAimMode::BaseFirst:
                             PriorTarget = RobotType::Base;
                             break;
                         case AutoAimMode::SentryFirst:
                             PriorTarget = RobotType::Sentry;
                             break;
                         default:
                             PriorTarget = RobotType::Negative;
                             break;
                     }
                     bool hasPriorTarget = false;
                     for(const auto& target : data.targets) {
                         if(target.id == PriorTarget) {
                             hasPriorTarget = true;
                             selected.targets.push_back(target);
                         }
                     }
                     if(!hasPriorTarget)
                         selected.targets = data.targets;
                     double minDisToImgCenter = std::numeric_limits<double>::max();
                     for(const auto& target : selected.targets) {
                         if(target.distToImgCenter < minDisToImgCenter) {
                             selected.selected = target;
                             minDisToImgCenter = target.distToImgCenter;
                         }
                     }

                     sendAll(set_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
                 } };
    }
};

HUB_REGISTER_CLASS(InfantryStrategy);
