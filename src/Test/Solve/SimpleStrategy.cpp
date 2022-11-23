#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedTarget.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <glm/gtc/random.hpp>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"

struct SimpleStrategySettings final {
    std::string aimType;
};

template <class Inspector>
bool inspect(Inspector& f, SimpleStrategySettings& x) {
    return f.object(x).fields(f.field("aimType", x.aimType).fallback("Car"));
}

class SimpleStrategy final : public HubHelper<caf::event_based_actor, SimpleStrategySettings, set_target_atom, set_outpost_atom> {
    Identifier mKey;

    const enum AimType { Car, Outpost } aimType;

public:
    SimpleStrategy(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ generateKey(this) }, aimType{
              magic_enum::enum_cast<AimType>(mConfig.aimType).value()
          } {}
    caf::behavior make_behavior() override {
        return { [&](detect_available_atom, GroupMask, Identifier key) {
                    ACTOR_PROTOCOL_CHECK(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
                    const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                    SelectedTarget selected;
                    selected.lastUpdate = data.lastUpdate;
                    selected.tfRobot2Gun = data.tfRobot2Gun;

                    auto minDistance = std::numeric_limits<double>::max();
                    for(auto& target : data.targets) {
                        if(const auto distance = glm::length(target.center.mVal); minDistance > distance) {
                            selected.selected = target;
                            minDistance = distance;
                        }
                    }

                    switch(aimType) {
                        case AimType::Car:
                            sendAll(set_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
                            break;
                        case AimType::Outpost:
                            sendAll(set_outpost_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
                            break;
                        default:
                            break;
                    }
                },
                 [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); } };
    }
};

HUB_REGISTER_CLASS(SimpleStrategy);
