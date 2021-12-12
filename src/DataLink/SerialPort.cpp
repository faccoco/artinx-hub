#include "AsyncSerial/BufferedAsyncSerial.h"
#include "BlackBoard.hpp"
#include "Hub.hpp"
#include "Packet.hpp"
#include "PostureData.hpp"
#include "Utility.hpp"
#include <boost/circular_buffer.hpp>
#include <caf/event_based_actor.hpp>

struct SerialPortSettings final {
    std::string devPath;
    uint32_t baudRate;
};

template <class Inspector>
bool inspect(Inspector& f, SerialPortSettings& x) {
    return f.object(x).fields(f.field("devPath", x.devPath), f.field("baudRate", x.baudRate));
}

class SerialPort final : public HubHelper<caf::event_based_actor, SerialPortSettings, update_head_atom, update_posture_atom> {
    constexpr static size_t bufferLen = 1024;
    constexpr static size_t headerLen = 5;

    BufferedAsyncSerial::Ptr mSerialPort;
    std::thread mThread;

    Identifier mKey;

    uint16_t mExpectedLen;
    std::array<uint8_t, bufferLen> mPacketBuffer;
    size_t mPacketLen;
    std::array<uint8_t, headerLen> mHeaderBuffer;
    size_t mHeaderLen;
    bool mCheckingHeader;

    void receive() {
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

    void handlePacket(uint16_t id) {
        switch(id) {
            case(GimbalFdbPacket::id): {
                BlackBoard::instance().updateSync(mKey, GimbalFdbPacket::receive(mPacketBuffer));
                sendAll(update_head_atom_v, mKey);
                sendAll(update_posture_atom_v, mKey);
                break;
            }
        }
    }

public:

    SerialPort(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mSerialPort(std::make_unique<BufferedAsyncSerial>()), mKey{ typeid(SerialPort).hash_code() },
          mCheckingHeader(false) {
        const auto [devPath, baudRate] = mConfig;
        mSerialPort->open(devPath, baudRate);
        mThread = std::thread{ [this]() {
            while(globalStatus == RunStatus::running) {
                BlackBoard::instance().updateSync(mKey, PostureData());
                receive();
            }
        } };
    }

    ~SerialPort() override {
        mSerialPort.release()->close();
        mThread.detach();
    }

    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [this](set_target_info_atom, double yawAngle, double pitchAngle, bool isFire) {
                     GimbalSetPacket gimbalSetPacket{static_cast<float>(yawAngle), static_cast<float>(pitchAngle), isFire};
                     mSerialPort->write(reinterpret_cast<const char*>(gimbalSetPacket.buffer.data()), GimbalSetPacket::size);
                 } };
    }
};

HUB_REGISTER_CLASS(SerialPort);
