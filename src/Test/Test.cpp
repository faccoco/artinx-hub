#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <cstdint>

class Input final : public HubHelper<caf::event_based_actor, void> {
public:
    Input(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
            for(int32_t i = 0; i < 10; ++i)
                for(int32_t j = 0; j < 10; ++j)
                    sendAll(i, j);
        } };
    }
};

HUB_REGISTER_CLASS(Input);

class Output final : public HubHelper<caf::event_based_actor, void> {
public:
    Output(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [](start_atom) {},
                 [this](const int32_t a, const int32_t b) {
                     const auto res = a + b;
                     caf::aout(this) << a << " + " << b << " = " << res << std::endl;
                 } };
    }
};

HUB_REGISTER_CLASS(Output);
