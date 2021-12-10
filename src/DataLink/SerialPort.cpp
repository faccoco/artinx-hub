#include "AsyncSerial/BufferedAsyncSerial.h"
#include "BlackBoard.hpp"
#include "Crc.hpp"
#include "Hub.hpp"
#include "Packet.hpp"
#include "PostureData.hpp"
#include "Utility.hpp"
#include <boost/circular_buffer.hpp>
#include <caf/event_based_actor.hpp>

struct SerialPortSettings final {
    std::string devPath;
    unsigned int baudRate;
};

template <class Inspector>
bool inspect(Inspector& f, SerialPortSettings& x) {
    return f.object(x).fields(f.field("devPath", x.devPath), f.field("baudRate", x.baudRate));
}

class SerialPort final : public HubHelper<caf::event_based_actor, SerialPortSettings, update_head_atom, update_posture_atom> {
public:
    constexpr static size_t bufferLen = 1024;
    constexpr static size_t headerLen = 5;

private:

    BufferedAsyncSerial::Ptr mSerialPort;
    std::thread mThread;

    Identifier mKey;

    uint16_t mExpectedLen;
    std::array<uint8_t, bufferLen> mPacketBuffer;
    size_t mPacketLen;
    std::array<uint8_t, headerLen> mHeaderBuffer;
    size_t mHeaderLen;
    bool mCheckingHeader;
    bool mStartFlag = false;

    GimbalSetPacket mSetPacket;

    void send() {
        mSetPacket.serialize();
        mSerialPort->write(reinterpret_cast<char*>(mSetPacket.buffer.data()), GimbalSetPacket::size());
    }

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
            case(0x0A): {
                BlackBoard::instance().updateSync(mKey, GimbalFdbPacket::handle(mPacketBuffer));
                break;
            }
        }
    }

public:

    SerialPort(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mSerialPort(std::make_unique<BufferedAsyncSerial>()), mKey{ typeid(SerialPort).hash_code() },
          mCheckingHeader(false) {
        const auto [devPath, baudRate] = mConfig;
        BlackBoard::instance().updateSync(mKey, GimbalFdbPacket());
        BlackBoard::instance().updateSync(mKey, GimbalSetPacket());
        mSerialPort->open(devPath, baudRate);
        mThread = std::thread{ [this]() {
            while(globalStatus == RunStatus::running) {
                BlackBoard::instance().updateSync(mKey, PostureData());
                if(mStartFlag) {
                    sendAll(update_head_atom_v, mKey);
                    sendAll(update_posture_atom_v, mKey);
                }
                receive();
            }
        } };
    }

    ~SerialPort() override {
        mSerialPort.release()->close();
        mThread.detach();
    }

    caf::behavior make_behavior() override {
        return { [this](start_atom) { mStartFlag = true; },
                 [this](set_target_posture_atom, double yawAngle, double pitchAngle) {
                     mSetPacket.yaw = static_cast<float>(yawAngle);
                     mSetPacket.pitch = static_cast<float>(pitchAngle);
                     send();
                 },
                 [this](shoot_atom, bool ifShoot) { mSetPacket.isFire = ifShoot; } };
    }
};

HUB_REGISTER_CLASS(SerialPort);
