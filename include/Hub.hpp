#pragma once
#include "Common.hpp"
#include "Config.hpp"
#include "DataDesc.hpp"
#include "Timer.hpp"

#include "SuppressWarningBegin.hpp"

#include "spdlog/async.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"

#include <atomic>
#include <caf/actor_addr.hpp>
#include <caf/actor_system.hpp>
#include <caf/config_value.hpp>

#include "SuppressWarningEnd.hpp"

#include <caf/event_based_actor.hpp>
#include <caf/fwd.hpp>
#include <caf/inspector_access.hpp>
#include <caf/result.hpp>
#include <caf/scheduled_actor.hpp>
#include <cassert>
#include <chrono>
#include <functional>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <type_traits>

using namespace std::literals;

class NodeFactory final : Unmovable {
    std::unordered_map<std::string, std::function<caf::actor(caf::actor_system&, const HubConfig&, std::string)>> mClasses{};

public:
    void addNodeType(std::string name,
                     std::function<caf::actor(caf::actor_system&, const HubConfig&, std::string)> spawnFunction);
    caf::actor buildNode(caf::actor_system& system, const std::string& name, const HubConfig& config);
    static NodeFactory& get();
};

namespace detail {
    void registerComponent(const char* name,
                           std::function<caf::actor(caf::actor_system&, const HubConfig&, std::string)> spawnFunction);

    template <typename NodeType>
    class HubClassRegister final : Unmovable {
    public:
        HubClassRegister() {
            registerComponent(typeid(NodeType).name(),
                              [](caf::actor_system& system, const HubConfig& config, std::string name) -> caf::actor {
                                  return system.spawn<NodeType>(config, name);
                              });
        }
    };

    std::vector<std::string> parseSucceed(const HubConfig& config, std::string_view name);

    std::vector<std::pair<caf::actor_addr, GroupMask>> parseSucceed(caf::actor_system& system,
                                                                    const std::vector<std::string>& succeed);
}  // namespace detail
#define HUB_REGISTER_CLASS(CLASS_NAME) static detail::HubClassRegister<CLASS_NAME> HubClassRegister##CLASS_NAME

template <typename T, typename Config, typename... Succeed>
class HubHelper : public T {
    static_assert(std::is_base_of_v<caf::abstract_actor, T>);

    template <typename Label>
    struct SucceedAddress final {
        std::vector<std::string> destName;
        std::vector<std::pair<caf::actor_addr, GroupMask>> addrVal;
        std::atomic_bool needReload{ true };
        SucceedAddress() = default;
        SucceedAddress(std::vector<std::string>&& names) : destName(names) {}
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
    static constexpr size_t mDestSize = sizeof...(Succeed);

    template <typename Atom>
    const auto& getDest() {
        auto& destInfo = std::get<SucceedAddress<Atom>>(mDest);
        if(destInfo.needReload.load(std::memory_order_consume)) {
            destInfo.addrVal = detail::parseSucceed(this->system(), destInfo.destName);
            destInfo.needReload.store(false, std::memory_order_release);
        }
        return destInfo.addrVal;
    }

    template <typename Dst, typename... Dsts>
    void buildDest(const HubConfig& config) {
        std::get<SucceedAddress<Dst>>(mDest).destName = std::move(detail::parseSucceed(config, typeid(Dst).name()));
        if constexpr(sizeof...(Dsts) > 0)
            buildDest<Dsts...>(config);
    }

    template <typename Dst, typename... Dsts>
    void setAddressReload(bool flag) {
        std::get<SucceedAddress<Dst>>(mDest).needReload.store(flag, std::memory_order_release);
        if constexpr(sizeof...(Dsts) > 0)
            setAddressReload<Dsts...>(flag);
    }

    void reloadConfig() {
        if constexpr(!std::is_void_v<Config>) {
            if(auto allConfig = ConfigHelper::instance().getConfig().to_dictionary()) {
                if(auto iter = allConfig.value().find(mNodeName); iter != allConfig.value().end()) {
                    if(auto configOpt = caf::get_as<Config>(iter->second)) {
                        mConfig = std::move(configOpt.value());
                    } else {
                        logError("Parse node config file failed!");
                    }
                } else {
                    logError(fmt::format("Can not find config for exist node {}", mNodeName));
                }
            } else {
                throw std::runtime_error("Parse node config file failed!");
            }
        }
    }

protected:
    std::conditional_t<std::is_void_v<Config>, char, Config> mConfig;
    GroupMask mGroupMask;
    const std::string mNodeName;

    template <typename Self>
    static Identifier generateKey(Self* thisPointer) {
        return { typeid(Self).hash_code() ^ reinterpret_cast<uintptr_t>(thisPointer) };
    }

public:
    HubHelper(caf::actor_config& base, const HubConfig& config, std::string name)
        : T{ base }, mDest{}, mNodeName{ std::move(name) } {
        if constexpr(mDestSize != 0)
            buildDest<Succeed...>(config);
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
        this->set_default_handler([&](caf::scheduled_actor* actor, caf::message& msg) {
            if(msg.size() != 1)
                return caf::print_and_drop(actor, msg);
            switch(msg.type_at(0)) {
                case caf::type_id<reload_address_atom>::value:
                    if constexpr(mDestSize != 0)
                        setAddressReload<Succeed...>(true);
                    break;
                case caf::type_id<reload_config_atom>::value:
                    reloadConfig();
                    break;
                default:
                    return caf::print_and_drop(actor, msg);
            }
            return caf::skippable_result{};
        });
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

    static void logInfoBoth(const std::string_view& msg) {
        logInfo(msg);
        visualLog(msg);
    }
};
// namespace TypeHelper {
// enum ConfigType { INT = 0, FLOAT = 1, DOUBLE = 2, STRING = 3, VECTOR = 4 };
// std::string parseTypeInfo(std::type_info info);

// template <typename T>
// std::string typeName() {
// #if defined(__clang__)
// std::string FunName = __PRETTY_FUNCTION__;
// size_t begPos = FunName.find("T = ");
// size_t endPos = FunName.find(']', begPos);
// begPos += 4;
// #elif defined(__GNUC__)
// std::string FunName = __PRETTY_FUNCTION__;
// size_t begPos = FunName.find("T = ");
// size_t endPos = FunName.find(';', begPos);
// begPos += 4;
// #elif defined(_MSC_VER)
// static_assert(false, "I don't want to use MSVC anymore. Please complete this by yourself if you want to use MSVC.");
// #endif
// return FunName.substr(begPos, endPos - begPos);
//}
//}  // namespace TypeHelper
