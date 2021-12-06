#pragma once
#include "Common.hpp"
#include <caf/actor_system.hpp>
#include <caf/config_value.hpp>
#include <chrono>
#include <functional>
#include <string_view>

using namespace std::literals;

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

    std::vector<std::string> parseSucceed(const HubConfig& config, const std::string& name);
    std::vector<caf::actor_addr> parseSucceed(caf::actor_system& system, const std::vector<std::string>& succeed);
}  // namespace detail

template <typename T, typename Config, typename... Succeed>
class HubHelper : public T {
    static_assert(std::is_base_of_v<caf::abstract_actor, T>);

    template <typename Label>
    struct SucceedAddress final {
        std::variant<std::vector<std::string>, std::vector<caf::actor_addr>> val;
    };

    std::tuple<SucceedAddress<Succeed>...> mDest;

protected:
    std::conditional_t<std::is_void_v<Config>, char, Config> mConfig;

public:
    HubHelper(caf::actor_config& base, const HubConfig& config)
        : T{ base }, mDest{ SucceedAddress<Succeed>{ detail::parseSucceed(config, typeid(Succeed).name()) }... } {
        if constexpr(!std::is_void_v<Config>) {
            auto configValue = caf::get_as<Config>(config);
            if(configValue) {
                mConfig = std::move(configValue.value());
            } else {
                CAF_LOG_ERROR("Bad config for " + std::string{ typeid(T).name() });
                // TODO: terminate & output error
            }
        }
    }
    // TODO: static type check
    template <typename Atom, typename... Args>
    void sendAll(Atom atom, Args&&... args) {
        auto& dest = std::get<SucceedAddress<Atom>>(mDest).val;
        if(dest.index() == 0)
            dest = detail::parseSucceed(this->system(), std::get<0>(dest));
        for(auto&& address : std::get<1>(dest))
            this->send(caf::actor_cast<caf::actor>(address), atom, args...);
    }
};
