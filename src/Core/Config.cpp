#include "Config.hpp"
#include "Common.hpp"
#include <caf/config_value.hpp>
#include <exception>
#include <fmt/core.h>
#include <fstream>
#include <mutex>
#include <string>

void ConfigHelper::loadConfig() {
    std::ifstream in{ mConfigPath, std::ios::in | std::ios::binary };
    const auto size = in.seekg(0, std::ios::end).tellg();
    mConfigData = std::string(static_cast<size_t>(size), '\0');
    in.seekg(0, std::ios::beg).read(mConfigData.data(), size);
}

void ConfigHelper::parseConfig() {
    mConfig = caf::config_value::parse(mConfigData).value();
};

void ConfigHelper::writeConfig() {
    std::lock_guard guard(mutex);
    std::ofstream out{ mConfigPath };
    out << mConfigData;
}

bool ConfigHelper::reloadConfig() noexcept {
    std::lock_guard guard{ mutex };
    try {
        loadConfig();
        parseConfig();
    } catch(std::exception& e) {
        logError(e.what());
        return false;
    }
    return true;
}

bool ConfigHelper::updateConfigData(std::string&& configData) noexcept {
    std::lock_guard guard(mutex);
    std::swap(configData, mConfigData);
    try {
        parseConfig();
    } catch(std::exception& e) {
        logError(fmt::format("Failed to update config: {}", e.what()));
        std::swap(configData, mConfigData);
        return false;
    }
    return true;
}

bool ConfigHelper::updateConfigPath(std::string_view path) noexcept {
    std::string originPath = std::move(mConfigPath);
    mConfigPath = path;
    try {
        reloadConfig();
    } catch(std::exception& e) {
        logError(e.what());
        std::swap(originPath, mConfigPath);
        return false;
    }
    return true;
}

HubConfig ConfigHelper::getConfig() {
    std::lock_guard guard{ mutex };
    return mConfig;
}

std::string ConfigHelper::getRaw() noexcept {
    std::lock_guard guard(mutex);
    return mConfigData;
}

ConfigHelper& ConfigHelper::instance() {
    static ConfigHelper instant;
    return instant;
}
