#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
class FakeSerialPort final : public HubHelper<caf::event_based_actor, void, ore_instructions_atom> {
    Identifier mKey;

public:
    FakeSerialPort(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(FakeSerialPort).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) { sendAll(ore_instructions_atom_v, true); },
                 [&](ore_detect_available_atom, double angle, Identifier key) { CAF_LOG_INFO(fmt::format("{}", angle)); } };
    }
};
HUB_REGISTER_CLASS(FakeSerialPort);
