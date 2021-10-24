#include "DataDesc.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <caf/actor_system.hpp>
#include <caf/event_based_actor.hpp>
#include <cstdint>

class DaemonActor final : public caf::event_based_actor {
public:
    DaemonActor(caf::actor_config& base, const std::vector<caf::actor>& actors) : event_based_actor{ base } {
        for(auto&& actor : actors)
            this->monitor(actor);
        this->set_down_handler([](const caf::down_msg& msg) {
            if(globalStatus == RunStatus::running)
                CAF_LOG_ERROR(caf::to_string(msg.reason));
            // TODO: resume actors
        });
    }

    caf::behavior make_behavior() override {
        return { [this](start_atom) {} };
    }
};

caf::actor createDaemonActor(caf::actor_system& sys, const std::vector<std::pair<std::string, caf::actor>>& actors) {
    std::vector<caf::actor> actorAddress;
    actorAddress.reserve(actors.size());
    for(auto& [name, actor] : actors)
        actorAddress.push_back(actor);

    return sys.spawn<DaemonActor>(std::move(actorAddress));
}
