#include "BlackBoard.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "Timer.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include <algorithm>
#include <caf/actor.hpp>
#include <caf/actor_addr.hpp>
#include <caf/actor_system.hpp>
#include <caf/config_value.hpp>
#include <caf/event_based_actor.hpp>
#include <caf/exit_reason.hpp>
#include <caf/scoped_actor.hpp>
#include <fmt/core.h>
#include <functional>
#include <string>

#include "SuppressWarningEnd.hpp"

class DaemonActor final : public caf::event_based_actor {
    mutable std::unordered_map<caf::actor_addr, std::string> mActors;
    const std::reference_wrapper<caf::actor_system> gSystem;
    const std::reference_wrapper<std::vector<std::pair<std::string, caf::actor>>> gActors;
    NodeFactory* gFactory;
    ConfigHelper* gConfigHelper;
    bool mStarted = false;

    void restartActor(const std::string& actorId) {
        auto& registry = gSystem.get().registry();
        const auto nodesConfig = gConfigHelper->getConfig().to_dictionary().value();
        caf::config_value actorConfig;
        for(auto&& [name, subConfig] : nodesConfig)
            if(name == actorId) {
                actorConfig = subConfig;
                break;
            }
        registry.erase(actorId);
        caf::scoped_actor caller{ gSystem.get() };
        std::for_each(gActors.get().begin(), gActors.get().end(), [&, instance = this, this](auto& actorInfo) {
            if(actorInfo.first == actorId) {
                caller->send_exit(actorInfo.second, caf::exit_reason::unreachable);
                actorInfo.second = std::move(gFactory->buildNode(gSystem.get(), actorId, actorConfig));
                instance->monitor(actorInfo.second);
                mActors.emplace(actorInfo.second.address(), actorId);
            }
        });
        for(const auto& actorInfo : gActors.get()) {
            caller->send(actorInfo.second, reload_address_atom_v);
        }
    }

public:
    DaemonActor(caf::actor_config& base, caf::actor_system& sys, std::vector<std::pair<std::string, caf::actor>>& actors)
        : event_based_actor{ base }, gSystem{ sys }, gActors{ actors }, gFactory{ &NodeFactory::get() },
          gConfigHelper{ &ConfigHelper::instance() } {
        for(auto&& [name, actor] : actors) {
            this->monitor(actor);
            mActors.emplace(actor.address(), name);
        }
        set_down_handler([this](const caf::down_msg& msg) {
            if(globalStatus != RunStatus::running)
                return;

            const auto actorName = mActors[msg.source];
            mActors.erase(msg.source);
            logError(fmt::format("Actor {} down: {}", actorName, caf::to_string(msg.reason)));
            HubLogger::visualLog(fmt::format("Actor {} down: {}", actorName, caf::to_string(msg.reason)));
            restartActor(actorName);
        });
        // Timer::instance().addTimer(address(), 500ms);
    }

    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                    mStarted = true;
                },
                 /*                 [this](timer_atom) {
                                      ACTOR_PROTOCOL_CHECK(timer_atom);
                                      if(!mStarted)
                                          return;
                                  },*/
                 [](monitor_response_atom) { ACTOR_PROTOCOL_CHECK(monitor_response_atom); } };
    }
};

caf::actor createDaemonActor(caf::actor_system& sys, std::vector<std::pair<std::string, caf::actor>>& actors) {
    return sys.spawn<DaemonActor>(sys, actors);
}
