#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedTarget.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/random.hpp>
#include <queue>

class SimpleStrategy final : public HubHelper<caf::event_based_actor, void, set_target_atom> {
    Identifier mKey;

public:
    SimpleStrategy(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(SimpleStrategy).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [&](detect_available_atom, Identifier key) {
                    const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                    SelectedTarget selected;
                    selected.lastUpdate = data.lastUpdate;
                    auto minDistance = std::numeric_limits<double>::max();
                    for(auto& target : data.targets) {
                        const auto distance = glm::length(target.center.raw());
                        if(minDistance > distance) {
                            selected.selected = target;
                            minDistance = distance;
                        }
                    }
                    BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected);
                    sendAll(set_target_atom_v, mKey);
                },
                 [](start_atom) {} };
    }
};

HUB_REGISTER_CLASS(SimpleStrategy);
