#include "Config.hpp"
#include <caf/config_value.hpp>
#include <fstream>
#include <mutex>
#include <string>

void ConfigHelper::loadConfig() {
    std::lock_guard guard(mutex);
    std::ifstream in{ configPath, std::ios::in | std::ios::binary };
    const auto size = in.seekg(0, std::ios::end).tellg();
    configData = std::string(static_cast<size_t>(size), '\0');
    in.seekg(0, std::ios::beg).read(configData.data(), size);
}

void ConfigHelper::setPath(std::string_view path) {
    configPath = path;
}

void ConfigHelper::setConfig(std::string_view src) {
    configData = src;
}

void ConfigHelper::writeConfig() {
    std::lock_guard guard(mutex);
    std::ofstream out{ configPath };
    out << configData;
}

void ConfigHelper::parseConfig() {
    config = caf::config_value::parse(configData).value();
};

std::string ConfigHelper::getRaw() {
    std::lock_guard guard(mutex);
    return configData;
}

void ConfigHelper::updateConfig() {
    loadConfig();
    parseConfig();
}

HubConfig ConfigHelper::updateConfig(std::string_view path) {
    configPath = path;
    updateConfig();
    return config;
}
HubConfig ConfigHelper::getConfig() {
    std::lock_guard guard{ mutex };
    return config;
}

ConfigHelper& ConfigHelper::instance() {
    static ConfigHelper instant;
    return instant;
}
