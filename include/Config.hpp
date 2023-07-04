#pragma once
#include "Common.hpp"
#include "DataDesc.hpp"
#include <caf/config_value.hpp>
#include <functional>
#include <mutex>

using HubConfig = caf::config_value;

class ConfigHelper : Uncopyable {
    std::string configPath;
    std::string configData;
    HubConfig config;

    ConfigHelper() = default;
    void loadConfig();
    void parseConfig();
    void updateConfig();

public:
    std::mutex mutex;

    static ConfigHelper& instance();
    void setPath(std::string_view path);
    void setConfig(std::string_view src);
    void writeConfig();
    [[maybe_unused]] caf::config_value updateConfig(std::string_view path);
    std::string getRaw();
    HubConfig getConfig();
};

#define CONFIG_REFLECTION_MODULE(REFLECTION_FUNCTION)                                                     \
    [this](config_operation_atom, Identifier key) {                                                       \
        ACTOR_PROTOCOL_CHECK(config_operation_atom, std::string);                                         \
        sendAll(config_reflection_atom, Blackboard::instance().updateSync(mKey, std::move(this.getRaw))); \
    }
