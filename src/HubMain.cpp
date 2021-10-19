#include "BlackBoard.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "Timer.hpp"
#include "Utility.hpp"
#include <caf/actor_registry.hpp>
#include <caf/actor_system.hpp>
#include <caf/actor_system_config.hpp>
#include <caf/event_based_actor.hpp>
#include <caf/exec_main.hpp>
#include <caf/logger.hpp>
#include <caf/scoped_actor.hpp>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

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
    std::vector<std::string> parseSucceed(const HubConfig& config, const std::string& name) {
        std::string_view nameNormalized = name;
        if(const auto pos = nameNormalized.find_last_of(' '); pos != std::string::npos) {
            nameNormalized = nameNormalized.substr(pos + 1);
        }
        const auto attr = config.to_dictionary().value();
        const auto iter = attr.find(nameNormalized);
        if(iter == attr.cend())
            CAF_LOG_ERROR("Succeed " + std::string{ nameNormalized } + " is needed");

        const auto succeed = iter->second.to_list().value();
        std::vector<std::string> res;
        res.reserve(succeed.size());
        for(auto id : succeed) {
            res.push_back(caf::to_string(id));
        }
        return res;
    }
    std::vector<caf::actor> parseSucceed(caf::actor_system& system, const std::vector<std::string>& succeed) {
        const auto& registry = system.registry();
        std::vector<caf::actor> res;
        res.reserve(succeed.size());
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

    auto&& factory = NodeFactory::get();

    std::vector<caf::actor> actors;
    actors.reserve(nodes.size());

    for(auto&& [name, config] : nodes) {
        if(name == "global")
            continue;
        actors.push_back(factory.buildNode(system, name, nodes.find(name)->second));
    }

    return actors;
}

void createDaemonActor(caf::actor_system& sys, const std::vector<caf::actor>& actors);

RunStatus globalStatus = RunStatus::running;
static std::mutex globalMutex;
static std::condition_variable globalCV;

void terminateSystem(caf::local_actor& actor, const bool success) {
    globalStatus = success ? RunStatus::normalExit : RunStatus::failureExit;
    /*
    for(auto& [name, address] : actor.system().registry().named_actors())
        actor.send_exit(address, caf::make_error(ExitCode::finished));
    */
    globalCV.notify_one();
}

void caf_main(caf::actor_system& system, const caf::actor_system_config& config) {
    CAF_LOG_INFO("Initializing");
    Timer::instance().bindSystem(system);

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
    BlackBoard::instance().updateSync({}, caf::get_as<GlobalSettings>(pipelineConfig.to_dictionary().value()["global"]).value());
    const auto actors = buildPipeline(system, pipelineConfig);

    createDaemonActor(system, actors);

    // std::this_thread::sleep_for(3s);

    caf::scoped_actor caller{ system };
    for(auto&& actor : actors) {
        caller->send(actor, start_atom_v);
    }
    CAF_LOG_INFO("ArtinxHub Started");

    {
        std::unique_lock<std::mutex> lock{ globalMutex };
        globalCV.wait(lock, [] { return globalStatus != RunStatus::running; });
    }

    // system.await_all_actors_done();
    CAF_LOG_INFO("ArtinxHub Finished");

    std::quick_exit(globalStatus == RunStatus::normalExit ? EXIT_SUCCESS : EXIT_FAILURE);
    // FIXME: stop all actors normally
    // return globalStatus == RunStatus::normalExit ? EXIT_SUCCESS : EXIT_FAILURE;
}

CAF_MAIN(caf::id_block::ArtinxHub)
