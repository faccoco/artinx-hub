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
#include <cctype>
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

template <typename String>
static void demangle(String& typeName) {
#ifdef ARTINXHUB_WINDOWS
    // For MSVC
    if(const auto pos = typeName.find_last_of(' '); pos != String::npos)
        typeName = typeName.substr(pos + 1);
#else
    // For GCC/Clang
    while(std::isdigit(typeName.front())) {
        typeName = typeName.substr(1);
    }
#endif  // ARTINXHUB_WINDOWS
}

class NodeFactory final : Unmovable {
    std::unordered_map<std::string, std::function<caf::actor(caf::actor_system&, const HubConfig&)>> mClasses{};

public:
    void addNodeType(std::string name, std::function<caf::actor(caf::actor_system&, const HubConfig&)> spawnFunction) {
        demangle(name);
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
        demangle(nameNormalized);
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
    std::vector<caf::actor_addr> parseSucceed(caf::actor_system& system, const std::vector<std::string>& succeed) {
        const auto& registry = system.registry();
        std::vector<caf::actor_addr> res;
        res.reserve(succeed.size());
        for(auto id : succeed) {
            res.push_back(registry.get<caf::actor_addr>(caf::to_string(id)));
        }
        return res;
    }
}  // namespace detail

std::vector<std::pair<std::string, caf::actor>> buildPipeline(caf::actor_system& system, const HubConfig& config) {
    const auto nodes = config.to_dictionary().value();
    std::unordered_map<std::string, uint32_t> idMap;
    std::vector<std::tuple<uint32_t, std::string, std::vector<uint32_t>>> reference;
    reference.reserve(nodes.size());

    auto&& factory = NodeFactory::get();

    std::vector<std::pair<std::string, caf::actor>> actors;
    actors.reserve(nodes.size());

    for(auto&& [name, config] : nodes) {
        if(name == "global")
            continue;
        actors.push_back({ name, factory.buildNode(system, name, nodes.find(name)->second) });
    }

    return actors;
}

caf::actor createDaemonActor(caf::actor_system& sys, const std::vector<std::pair<std::string, caf::actor>>& actors);

RunStatus globalStatus = RunStatus::running;
static std::mutex globalMutex;
static std::condition_variable globalCV;

void terminateSystem(caf::local_actor& actor, const bool success) {
    globalStatus = success ? RunStatus::normalExit : RunStatus::failureExit;
    globalCV.notify_one();
}

int caf_main(caf::actor_system& system, const caf::actor_system_config& config) {
    CAF_LOG_INFO("Initializing");
    Timer::instance().bindSystem(system);

    const fs::path logPath{ "./logs" };
    if(!fs::exists(logPath)) {
        fs::create_directory(logPath);
    }

    const auto [argc, argv] = config.c_args_remainder();
    auto args = "Command Arguments: "s;
    for(int idx = 1; idx < argc; ++idx) {
        args += argv[idx];
        args += ' ';
    }
    CAF_LOG_INFO(args);

    if(argc != 2 || !fs::exists(argv[1])) {
        CAF_LOG_ERROR("Bad Config");
        return EXIT_FAILURE;
    }

    const auto configData = loadConfig(argv[1]);
    const auto pipelineConfig = caf::config_value::parse(configData).value();
    BlackBoard::instance().updateSync({}, caf::get_as<GlobalSettings>(pipelineConfig.to_dictionary().value()["global"]).value());

    {
        const auto actors = buildPipeline(system, pipelineConfig);
        const auto daemon = createDaemonActor(system, actors);

        caf::scoped_actor caller{ system };
        for(auto&& [name, actor] : actors) {
            caller->send(actor, start_atom_v);
        }
        CAF_LOG_INFO("ArtinxHub Started");

        {
            std::unique_lock<std::mutex> lock{ globalMutex };
            globalCV.wait(lock, [] { return globalStatus != RunStatus::running; });
        }

        CAF_LOG_INFO("ArtinxHub Finished");

        for(auto& [name, actor] : actors) {
            system.registry().erase(name);
            caller->send_exit(actor, caf::exit_reason::user_shutdown);
        }
        caller->send_exit(daemon, caf::exit_reason::user_shutdown);
    }

    system.await_all_actors_done();

    return globalStatus == RunStatus::normalExit ? EXIT_SUCCESS : EXIT_FAILURE;
}

CAF_MAIN(caf::id_block::ArtinxHub)
