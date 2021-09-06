#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/actor_registry.hpp>
#include <caf/actor_system.hpp>
#include <caf/actor_system_config.hpp>
#include <caf/event_based_actor.hpp>
#include <caf/exec_main.hpp>
#include <caf/logger.hpp>
#include <caf/scoped_actor.hpp>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;
using namespace std::literals;

static std::string loadConfig(const char* path) {
    std::ifstream in{ path, std::ios::in | std::ios::binary };
    const auto size = in.seekg(0, std::ios::end).tellg();
    std::string res(static_cast<size_t>(size), '\0');
    in.seekg(0, std::ios::beg).read(res.data(), size);
    return res;
}

class NodeFactory final : Unmovable {
    std::unordered_map<std::string, std::function<caf::actor(caf::actor_system&, const HubConfig&)>> mClasses{};

public:
    void addNodeType(std::string name, std::function<caf::actor(caf::actor_system&, const HubConfig&)> spawnFunction) {
        if(name.find("class ") == 0)
            name = name.substr(6);
        if(!mClasses.emplace(std::move(name), std::move(spawnFunction)).second) {
            const auto error = "Multiple definition of node type " + name;
            CAF_RAISE_ERROR(error.c_str());
        }
    }
    caf::actor buildNode(caf::actor_system& system, const std::string& name, const HubConfig& config) {
        CAF_LOG_INFO("Build node " + name);
        const auto attr = config.to_dictionary().value();
        const auto nodeTypeName = caf::to_string(attr.find("type"sv)->second);
        const auto iter = mClasses.find(nodeTypeName);
        if(iter == mClasses.cend()) {
            const auto error = "Undefined Node Type " + nodeTypeName;
            CAF_RAISE_ERROR(error.c_str());
        }
        auto actor = iter->second(system, config);
        system.registry().put(name, actor);
        return actor;
    }
    static NodeFactory& get() {
        static NodeFactory instance;
        return instance;
    }
};

namespace detail {
    void registerComponent(const char* name, std::function<caf::actor(caf::actor_system&, const HubConfig&)> spawnFunction) {
        NodeFactory::get().addNodeType(std::string{ name }, std::move(spawnFunction));
    }
    std::vector<caf::actor> parseSucceed(caf::actor_system& system, const HubConfig& config) {
        const auto attr = config.to_dictionary().value();
        const auto iter = attr.find("succeed"sv);
        if(iter == attr.cend())
            return {};
        const auto& registry = system.registry();
        std::vector<caf::actor> res;
        const auto succeed = iter->second.to_list().value();
        for(auto id : succeed) {
            res.push_back(registry.get<caf::actor>(caf::to_string(id)));
        }
        return res;
    }
}  // namespace detail

std::vector<caf::actor> buildPipeline(caf::actor_system& system, const HubConfig& config) {
    const auto nodes = config.to_dictionary().value();
    std::unordered_map<std::string, uint32_t> idMap;
    std::vector<std::tuple<uint32_t, std::string, std::vector<uint32_t>>> reference;
    reference.reserve(nodes.size());

    for(auto&& [name, config] : nodes) {
        idMap.emplace(name, static_cast<uint32_t>(reference.size()));
        reference.push_back({ 0, name, {} });
    }

    for(auto&& [name, config] : nodes) {
        const auto idx = idMap.find(name)->second;
        auto& linkCount = std::get<uint32_t>(reference[idx]);

        const auto attr = config.to_dictionary().value();
        const auto succeedIter = attr.find("succeed"sv);
        if(succeedIter == attr.cend())
            continue;

        const auto succeed = succeedIter->second.to_list().value();
        for(auto&& ref : succeed) {
            const auto refName = caf::to_string(ref);
            const auto iter = idMap.find(refName);
            if(iter == idMap.cend()) {
                const auto error = "Undefined Node " + refName;
                CAF_RAISE_ERROR(error.c_str());
            }
            ++linkCount;
            auto& link = std::get<std::vector<uint32_t>>(reference[iter->second]);
            link.push_back(idx);
        }
    }

    auto&& factory = NodeFactory::get();

    std::vector<caf::actor> actors;
    actors.reserve(nodes.size());

    while(true) {
        bool sideEffects = false;

        for(auto&& [ref, name, link] : reference) {
            if(ref == 0) {
                actors.push_back(factory.buildNode(system, name, nodes.find(name)->second));
                ref = std::numeric_limits<uint32_t>::max();
                sideEffects = true;

                for(auto dep : link)
                    --std::get<uint32_t>(reference[dep]);
            }
        }

        if(!sideEffects) {
            for(auto&& [ref, name, link] : reference) {
                if(ref != std::numeric_limits<uint32_t>::max()) {
                    CAF_RAISE_ERROR("Cycles detected.");
                }
            }
            break;
        }
    }

    return actors;
}

void caf_main(caf::actor_system& system, const caf::actor_system_config& config) {
    CAF_LOG_INFO("ArtinxHub Started");
    CAF_LOG_INFO("Initializing");
    const auto [argc, argv] = config.c_args_remainder();
    auto args = "Command Arguments: "s;
    for(int idx = 1; idx < argc; ++idx) {
        args += argv[idx];
        args += ' ';
    }
    CAF_LOG_INFO(args);

    if(argc != 2 || !fs::exists(argv[1])) {
        CAF_LOG_ERROR("Bad Config");
        return;
    }

    const auto configData = loadConfig(argv[1]);
    const auto pipelineConfig = caf::config_value::parse(configData).value();
    const auto actors = buildPipeline(system, pipelineConfig);

    std::this_thread::sleep_for(3s);

    const caf::scoped_actor caller{ system };
    for(auto&& actor : actors) {
        caller->send(actor, start_atom_v);
    }
}

CAF_MAIN(caf::id_block::ArtinxHub)
