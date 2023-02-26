#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedBots.hpp"
#include "Hub.hpp"
#include "Packet.hpp"
#include "PostureData.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include "AsyncSerial/BufferedAsyncSerial.h"
#include <caf/event_based_actor.hpp>
#include <deque>

#include "SuppressWarningEnd.hpp"

struct PosSynchronizationSettings final {
    std::string devPath;
    uint32_t baudRate;
};

template <class Inspector>
bool inspect(Inspector& f, PosSynchronizationSettings& x) {
    return f.object(x).fields(f.field("devPath", x.devPath), f.field("baudRate", x.baudRate));
}

class PosSynchronization final : public HubHelper<caf::event_based_actor, PosSynchronizationSettings, sync_position_atom> {
    constexpr static size_t bufferLen = 1024;
    constexpr static size_t headerLen = 5;
    constexpr static size_t sendBufferLen = 1024;

    bool started = false;

    BufferedAsyncSerial::Ptr mSerialPort;
    Identifier mKey;
    RadarPositionPacket mPosPacket;

    std::thread mThread;
    std::deque<SingleBotPos> mSendDeque;

    //    uint16_t mExpectedLen;
    //    std::array<uint8_t, bufferLen> mPacketBuffer;
    //    uint32_t mPacketLen;
    //    std::array<uint8_t, headerLen> mHeaderBuffer;
    //    uint32_t mHeaderLen;
    //    std::array<uint8_t, sendBufferLen> mSendBuffer;
    //    size_t mSendBufferLen;

    TimePoint lastReceivedTime, lastUpTargetTime, lastDownTargetTime;

    void receive() {}

    std::deque<Clock::rep> mLastFrames;

    void handlePacket(uint16_t id) {}

    void sendPacket() {}

public:
    PosSynchronization(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mSerialPort(std::make_unique<BufferedAsyncSerial>()), mKey{ generateKey(this) },
          mPosPacket(mSerialPort), mSendDeque(0) {
        mSerialPort->open(mConfig.devPath, mConfig.baudRate);
        lastReceivedTime = SynchronizedClock::instance().now();
        mThread = std::thread{ [this]() {
            while(globalStatus == RunStatus::running) {
                sendPacket();
                std::this_thread::sleep_for(0.75ms);
                //                mSendBufferLen += gimbalSetPacket.buffer.size();
            }
        } };
    }

    ~PosSynchronization() override {
        mSerialPort.release()->close();
        mThread.detach();
    }

    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                    started = true;
                },
                 [this](sync_position_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(sync_position_atom, BotsLocation);
                     auto data = BlackBoard::instance().get<BotsLocation>(key).value();
                     for(auto& bot : data.data)
                         mSendDeque.push_back({ bot.id, static_cast<float>(bot.x), static_cast<float>(bot.y) });
                 } };
    }
};

HUB_REGISTER_CLASS(PosSynchronization);
