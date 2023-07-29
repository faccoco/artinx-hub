#ifdef ARTINX_RADAR
#include "BlackBoard.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "RadarInfo.hpp"
#include "Utility.hpp"

#include <caf/actor_config.hpp>
#include <caf/behavior.hpp>
#include <caf/event_based_actor.hpp>
#include <condition_variable>
#include <fmt/format.h>
#include <mutex>
#include <random>
#include <string>
#include <thread>

class SerialPortTester final : public HubHelper<caf::event_based_actor, void, sync_position_atom> {
    Identifier mKey;
    std::mutex mMutex;
    std::condition_variable started;
    std::thread mThread;
    BotsPosition mPosition{};

    void genTestData() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<int> botNum(0, 10);
        std::uniform_int_distribution<uint16_t> botId(0, 15);
        std::uniform_real_distribution<float> botX(0.0, 15.0), botY(0.0, 15), botZ(0.0, 1);
        auto& data = mPosition.data;
        for(int i = 0; i < botNum(gen); ++i) {
            data.push_back({ botId(gen), botX(gen), botY(gen) });
            // logInfo(fmt::format("Bot ID: {} X: {} Y: {} Z: {}", std::to_string(data.back().id), std::to_string(data.back().x),
            // std::to_string(data.back().y), std::to_string(data.back().z)));
        }
    }

public:
    SerialPortTester(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, name }, mKey(generateKey(this)), mThread([this] {
              std::unique_lock<std::mutex> lock(mMutex);
              started.wait(lock);
              while(globalStatus == RunStatus::running) {
                  genTestData();
                  sendAll(sync_position_atom_v, BlackBoard::instance().updateSync(mKey, std::move(mPosition)));
                  mPosition = BotsPosition{};
                  std::this_thread::sleep_for(100ms);
              }
          }) {}

    ~SerialPortTester() {
        mThread.detach();
    }

    caf::behavior make_behavior() override {
        return { [this](start_atom) { started.notify_one(); } };
    }
};

HUB_REGISTER_CLASS(SerialPortTester);
#endif
