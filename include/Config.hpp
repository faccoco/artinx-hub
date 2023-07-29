#pragma once
#include "Common.hpp"
#include "DataDesc.hpp"
#include <caf/config_value.hpp>
#include <functional>
#include <mutex>

using HubConfig = caf::config_value;

enum class ConfigControl { ApplyConfig, WriteConfig, RestoreConfig, Nop };

class ConfigHelper : Uncopyable {
    std::string mConfigPath;
    std::string mConfigData;
    HubConfig mConfig;

    ConfigHelper() = default;
    void loadConfig();
    void parseConfig();

public:
    std::mutex mutex;

    static ConfigHelper& instance();
    void writeConfig();
    bool updateConfigData(std::string&& configData) noexcept;
    bool updateConfigPath(std::string_view path) noexcept;
    bool reloadConfig() noexcept;
    std::string getRaw() noexcept;
    HubConfig getConfig();
};

#define CONFIG_REFLECTION_MODULE(REFLECTION_FUNCTION)                                                     \
    [this](config_operation_atom, Identifier key) {                                                       \
        ACTOR_PROTOCOL_CHECK(config_operation_atom, std::string);                                         \
        sendAll(config_reflection_atom, Blackboard::instance().updateSync(mKey, std::move(this.getRaw))); \
    }
