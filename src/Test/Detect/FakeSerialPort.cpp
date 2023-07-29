#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>

#include "SuppressWarningEnd.hpp"

class FakeSerialPort final : public HubHelper<caf::event_based_actor, void> {
    Identifier mKey;

public:
    FakeSerialPort(caf::actor_config& base, const HubConfig& config, std::string name) : HubHelper{ base, config, name }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); } };
    }
};
HUB_REGISTER_CLASS(FakeSerialPort);
