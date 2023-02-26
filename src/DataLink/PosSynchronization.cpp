#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "Packet.hpp"
#include "PostureData.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include "AsyncSerial/BufferedAsyncSerial.h"
#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/gtc/matrix_transform.hpp>

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

    GimbalSetPacket gimbalSetPacket{};

    BufferedAsyncSerial::Ptr mSerialPort;
    std::thread mThread;

    Identifier mKey;

    bool started = false;

    uint16_t mExpectedLen;
    std::array<uint8_t, bufferLen> mPacketBuffer;
    uint32_t mPacketLen;
    std::array<uint8_t, headerLen> mHeaderBuffer;
    uint32_t mHeaderLen;
    bool mCheckingHeader;
    std::array<uint8_t, sendBufferLen> mSendBuffer;
    size_t mSendBufferLen;

    // float lastSpeedX = 0.0f, lastSpeedY = 0.0f;
    TimePoint lastReceivedTime, lastUpTargetTime, lastDownTargetTime;

    void receive() {
        if(!started)
            return;
        std::vector<char> vec = mSerialPort->read();
        for(uint8_t data : vec) {
            if(mPacketLen < bufferLen) {
                mPacketBuffer[mPacketLen++] = data;
                if(mPacketLen == mExpectedLen && Crc::VerifyCrc16CheckSum(mPacketBuffer.data(), mPacketLen)) {
                    handlePacket(mPacketBuffer[5]);
                }
            }

            if(mCheckingHeader) {
                mHeaderBuffer[mHeaderLen++] = data;
                if(mHeaderLen == 5) {
                    mCheckingHeader = false;
                    if(Crc::VerifyCrc8CheckSum(mHeaderBuffer.data(), mHeaderLen)) {
                        mExpectedLen = mHeaderBuffer[1] + 8;
                        std::copy(mHeaderBuffer.begin(), mHeaderBuffer.end(), mPacketBuffer.begin());
                        mPacketLen = 5;
                    }
                    mHeaderLen = 0;
                }
            }

            if(data == 0xA5) {
                mCheckingHeader = true;
                mHeaderLen = 0;
                mHeaderBuffer[mHeaderLen++] = data;
            }
        }
    }

    std::deque<Clock::rep> mLastFrames;

    void reportFrameRate(Clock::time_point timeStamp) {
        const auto current = timeStamp.time_since_epoch().count();
        mLastFrames.push_back(current);

        while(current - mLastFrames.front() > 1'000'000'000)
            mLastFrames.pop_front();

        const auto delta = std::max(static_cast<Clock::rep>(1), current - mLastFrames.front());
        const auto fps = (static_cast<double>(mLastFrames.size()) - 1.0) * 1e9 / static_cast<double>(delta);
        HubLogger::watch("ups", static_cast<uint32_t>(fps));
    }

    void handlePacket(uint16_t id) {}

    void sendPacket() {
        if(mSendBufferLen > sendBufferLen)
            mSendBufferLen = 0;
        if(mSendBufferLen == 0)
            return;
        // std::cout << mSendBufferLen << std::endl;
        mSerialPort->write(reinterpret_cast<char*>(mSendBuffer.data()), mSendBufferLen);
        mSendBufferLen = 0;
    }

public:
    PosSynchronization(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mSerialPort(std::make_unique<BufferedAsyncSerial>()), mKey{ generateKey(this) },
          mCheckingHeader(false) {
        mSerialPort->open(mConfig.devPath, mConfig.baudRate);
        lastReceivedTime = SynchronizedClock::instance().now();
        gimbalSetPacket.serialize();
        mThread = std::thread{ [this]() {
            while(globalStatus == RunStatus::running) {
                receive();
                sendPacket();
                std::this_thread::sleep_for(0.75ms);
                uint8_t targetBits = 0;
                if(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - lastUpTargetTime).count() < 500) {
                    targetBits |= 1;
                }
                if(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - lastDownTargetTime).count() < 500) {
                    targetBits |= 2;
                }
                HubLogger::watch("hasTargets", targetBits);
                gimbalSetPacket.setHasTargetBits(targetBits);
                gimbalSetPacket.serialize();
                gimbalSetPacket.buffer.copyToSendBuffer(mSendBuffer.data() + mSendBufferLen);
                mSendBufferLen += gimbalSetPacket.buffer.size();
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
                 [](sync_position_atom) {
                     ACTOR_PROTOCOL_CHECK(set_target_info_atom, GroupMask, Clock::rep, double, double, bool);
                 } };
    }
};

HUB_REGISTER_CLASS(PosSynchronization);
