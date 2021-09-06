#pragma once
#include "Common.hpp"
#include <caf/actor_system.hpp>
#include <caf/config_value.hpp>
#include <chrono>
#include <functional>
#include <string_view>

using namespace std::literals;

using Clock = std::chrono::high_resolution_clock;
using HubConfig = caf::config_value;

namespace detail {
    void registerComponent(const char* name, std::function<caf::actor(caf::actor_system&, const HubConfig&)> spawnFunction);

    template <typename NodeType>
    class HubClassRegister final : Unmovable {
    public:
        HubClassRegister() {
            registerComponent(typeid(NodeType).name(), [](caf::actor_system& system, const HubConfig& config) -> caf::actor {
                return system.spawn<NodeType>(config);
            });
        }
    };
#define HUB_REGISTER_CLASS(CLASS_NAME) static detail::HubClassRegister<CLASS_NAME> hubClassRegister##CLASS_NAME

    std::vector<caf::actor> parseSucceed(caf::actor_system& system, const HubConfig& config);
}  // namespace detail

template <typename T, typename Config, typename = std::enable_if_t<std::is_base_of_v<caf::abstract_actor, T>>>
class HubHelper : public T {
    std::vector<caf::actor> mDest;

protected:
    std::conditional_t<std::is_void_v<Config>, char, Config> mConfig;

public:
    HubHelper(caf::actor_config& base, const HubConfig& config)
        : T{ base }, mDest{ detail::parseSucceed(this->system(), config) } {
        if constexpr(!std::is_void_v<Config>) {
            mConfig = caf::get_as<Config>(config).value();
        }
    }
    template <typename... Args>
    void sendAll(Args&&... args) {
        for(auto&& address : mDest)
            this->send(address, args...);
    }
};
