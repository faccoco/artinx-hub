#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <cstdint>

class AngleSolver final : public HubHelper<caf::event_based_actor, void> {
public:
    AngleSolver(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {}, [this]() {} };
    }
};

HUB_REGISTER_CLASS(AngleSolver);
