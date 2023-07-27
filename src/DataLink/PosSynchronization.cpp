#ifdef ARTINX_RADAR
#include "BlackBoard.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "RadarInfo.hpp"
#include "SerialPort/Packet.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include "AsyncSerial/BufferedAsyncSerial.h"
#include <caf/event_based_actor.hpp>

#include "SuppressWarningEnd.hpp"

#include <deque>
#include <mutex>
#include <string>

struct PosSynchronizationSettings final {
    std::string devPath;
    uint32_t baudRate;
};

template <class Inspector>
bool inspect(Inspector& f, PosSynchronizationSettings& x) {
    return f.object(x).fields(f.field("devPath", x.devPath), f.field("baudRate", x.baudRate));
}

class PosSynchronization final : public HubHelper<caf::event_based_actor, PosSynchronizationSettings> {
    BufferedAsyncSerial::Ptr mSerialPort;
    std::mutex mMutex;
    std::condition_variable busy;

    MapMessage mMapMessage;
    std::deque<MapData> mSendQueue;
    std::thread mThread;

    void sendPacket() {
        std::lock_guard<std::mutex> guard(mMutex);
        if(mSendQueue.empty())
            return;
        mMapMessage.clear();
        mMapMessage.enBuffer(mSendQueue.front());
        mSerialPort->write(mMapMessage.data(), mMapMessage.size());
        mSendQueue.pop_front();
    }

public:
    PosSynchronization(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mSerialPort(std::make_unique<BufferedAsyncSerial>()), mMutex(), busy(), mThread([this]() {
              mSerialPort->open(mConfig.devPath, mConfig.baudRate);
              while(globalStatus == RunStatus::running) {
                  sendPacket();
                  std::this_thread::sleep_for(5ms);
              }
          }) {}

    ~PosSynchronization() override {
        mSerialPort.release()->close();
        mThread.detach();
    }

    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                    busy.notify_one();
                },
                 [this](sync_position_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(sync_position_atom, TypedIdentifier<BotsPosition>);
                     if(const auto data = BlackBoard::instance().get<BotsPosition>(key)) {
                         for(auto& bot : data.value().data)
                             mSendQueue.push_back({ bot.id, static_cast<float>(bot.x), static_cast<float>(bot.y) });
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(PosSynchronization);
#endif
