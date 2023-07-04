#pragma once
#include "Common.hpp"
#include "Config.hpp"
#include "Timer.hpp"

#include "SuppressWarningBegin.hpp"

#include "spdlog/async.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"

#include <caf/actor_system.hpp>
#include <caf/config_value.hpp>

#include "SuppressWarningEnd.hpp"

#include <cassert>
#include <chrono>
#include <functional>
#include <mutex>
#include <shared_mutex>
#include <string_view>
#include <type_traits>

using namespace std::literals;

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

    std::vector<std::string> parseSucceed(const HubConfig& config, std::string_view name);
    std::vector<std::pair<caf::actor_addr, GroupMask>> parseSucceed(caf::actor_system& system,
                                                                    const std::vector<std::string>& succeed);
}  // namespace detail

template <typename T, typename Config, typename... Succeed>
class HubHelper : public T {
    static_assert(std::is_base_of_v<caf::abstract_actor, T>);

    template <typename Label>
    struct SucceedAddress final {
        std::variant<std::vector<std::string>, std::vector<std::pair<caf::actor_addr, GroupMask>>> val;
    };

    template <typename Arg>
    static const Arg& wrap(const Arg& arg) noexcept {
        return arg;
    }

    template <typename... Arg>
    static const Identifier& wrap(const TypedIdentifier<Arg...>& arg) noexcept {
        return static_cast<const Identifier&>(arg);
    }

    std::tuple<SucceedAddress<Succeed>...> mDest;
    std::shared_mutex sMutex;

    template <typename Atom>
    const auto& getDest() {
        auto& dest = std::get<SucceedAddress<Atom>>(mDest).val;
        if(dest.index() == 0)
            dest = detail::parseSucceed(this->system(), std::get<0>(dest));
        return std::get<1>(dest);
    }

protected:
    std::conditional_t<std::is_void_v<Config>, char, Config> mConfig;
    GroupMask mGroupMask;

    template <typename Self>
    static Identifier generateKey(Self* thisPointer) {
        return { typeid(Self).hash_code() ^ reinterpret_cast<uintptr_t>(thisPointer) };
    }

public:
    HubHelper(caf::actor_config& base, const HubConfig& config)
        : T{ base }, mDest{ SucceedAddress<Succeed>{ detail::parseSucceed(config, typeid(Succeed).name()) }... } {
        if constexpr(!std::is_void_v<Config>) {
            if(auto configValue = caf::get_as<Config>(config)) {
                mConfig = std::move(configValue.value());
            } else {
                logError("Parse node config file failed!");
            }
        }

        const auto& dict = config.to_dictionary();
        if(const auto iter1 = dict->find("group_mask"); iter1 != dict->cend()) {
            mGroupMask = static_cast<uint32_t>(iter1->second.to_integer().value());
        } else if(const auto iter2 = dict->find("group_id"); iter2 != dict->cend()) {
            mGroupMask = 1U << static_cast<uint32_t>(iter2->second.to_integer().value());
        } else {
            mGroupMask = 1U;
        }
    }

    void reloadConfig() {
        if constexpr(!std::is_void_v<Config>) {
            auto& gConfig = ConfigHelper::instance();
            std::lock_guard guard(gConfig.mutex);
            if(auto configValue = caf::get_as<Config>(gConfig.getConfig())) {
                mConfig = std::move(configValue.value());
            } else {
                logError("Parse node config file failed!");
            }
        }
    }

    HubConfig getConfig() {
        std::shared_lock lock(sMutex);
        return mConfig;
    }

    template <typename Atom, typename... Args>
    void sendAll(Atom atom, Args&&... args) {
        ACTOR_PROTOCOL_CHECK(Atom, std::decay_t<Args>...);
        for(auto&& [address, mask] : getDest<Atom>())
            this->send(caf::actor_cast<caf::actor>(address), atom, wrap(std::forward<Args>(args))...);
    }

    template <typename Atom, typename... Args>
    void sendAllHighPriority(Atom atom, Args&&... args) {
        ACTOR_PROTOCOL_CHECK(Atom, std::decay_t<Args>...);
        for(auto&& [address, mask] : getDest<Atom>())
            this->template send<caf::message_priority::high>(caf::actor_cast<caf::actor>(address), atom,
                                                             wrap(std::forward<Args>(args))...);
    }

    template <typename Atom, typename... Args>
    void sendMasked(Atom atom, GroupMask mask, Args&&... args) {
        ACTOR_PROTOCOL_CHECK(Atom, std::decay_t<Args>...);
        for(auto&& [address, maskRhs] : getDest<Atom>())
            if(mask & maskRhs)
                this->send(caf::actor_cast<caf::actor>(address), atom, wrap(std::forward<Args>(args))...);
    }
};

class HubLogger final {
    static std::mutex mutex;
    static std::unordered_map<std::string, TimePoint> logs;
    static std::string prefix;
    static std::unordered_map<std::string_view, std::string> watches;

public:
    static void watch(const std::string& name, std::string_view log) {
        std::lock_guard guard{ mutex };
        watches[name] = log;
    }

    template <typename T>
    static void watch(std::string_view name, const T& log) {
        std::lock_guard guard{ mutex };
        if constexpr(std::is_convertible_v<std::decay_t<T>, std::string> ||
                     std::is_convertible_v<std::decay_t<T>, std::string_view>)
            watches[name] = log;
        else
            watches[name] = std::to_string(log);
    }

    static void removeWatch(const std::string& name) {
        std::lock_guard guard{ mutex };
        watches.erase(name);
    }
    static std::unordered_map<std::string_view, std::string> getwatches() {
        std::lock_guard guard{ mutex };
        return watches;
    };

    static void print(std::string_view log, const std::string& name, const int& interval) {
        std::lock_guard guard{ mutex };
        if(logs.find(name) != logs.end()) {
            if(std::chrono::duration_cast<std::chrono::milliseconds>(SynchronizedClock::instance().now() - logs[name]).count() <
               interval)
                return;
        }
        logs[name] = SynchronizedClock::instance().now();
        logInfo(log);
    }

    static void printDebugOnly(std::string_view log, const std::string& name, const int& interval) {
        std::lock_guard guard{ mutex };
#ifndef ARTINXHUB_DEBUG
        return;
#endif
        print(log, name, interval);
    }

    static void electricCtrlLog(const std::string_view msg) {
        static auto electricLogger = spdlog::rotating_logger_mt<spdlog::async_factory>(
            "ElectricLogger", fmt::format("data/logs/electric_log_{}.txt", prefix), 1024 * 1024 * 5, 200000);
        electricLogger->info(msg);
    }

    static void visualLog(const std::string_view msg) {
        static auto visualLogger = spdlog::rotating_logger_mt<spdlog::async_factory>(
            "VisualLogger", fmt::format("data/logs/visual_log_{}.txt", prefix), 1024 * 1024 * 5, 200000);
        visualLogger->info(msg);
    }
};
namespace TypeHelper {
    enum ConfigType { INT = 0, FLOAT = 1, DOUBLE = 2, STRING = 3, VECTOR = 4 };
    std::string parseTypeInfo(std::type_info info);

    template <typename T>
    std::string typeName() {
#if defined(__clang__)
        std::string FunName = __PRETTY_FUNCTION__;
        size_t begPos = FunName.find("T = ");
        size_t endPos = FunName.find(']', begPos);
        begPos += 4;
#elif define(__GNUC__)
        std::string FunName = __PRETTY_FUNCTION__;
        size_t begPos = FunName.find("T = ");
        size_t endPos = FunName.find(';', begPos);
        begPos += 4;
#elif defined(_MSC_VER)
        static_assert(false, "I don't want to use MSVC anymore. Please complete this by yourself if you want to use MSVC.");
#endif
        return FunName.substr(begPos, endPos - begPos);
    }
}  // namespace TypeHelper
