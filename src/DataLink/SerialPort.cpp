#include "AsyncSerial/BufferedAsyncSerial.h"
#include "BlackBoard.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "Packet.hpp"
#include "PostureData.hpp"
#include "Utility.hpp"
#include <boost/circular_buffer.hpp>
#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/gtc/matrix_transform.hpp>
#include <pthread.h>

struct SerialPortSettings final {
    std::string devPath;
    uint32_t baudRate;
    double headHeightOffset;
};

template <class Inspector>
bool inspect(Inspector& f, SerialPortSettings& x) {
    return f.object(x).fields(f.field("devPath", x.devPath), f.field("baudRate", x.baudRate),
                              f.field("headHeightOffset", x.headHeightOffset));
}

class SerialPort final : public HubHelper<caf::event_based_actor, SerialPortSettings, update_head_atom, update_posture_atom> {
    constexpr static size_t bufferLen = 1024;
    constexpr static size_t headerLen = 5;
    constexpr static size_t sendBufferLen = 1024;

    GimbalSetPacket gimbalSetPacket{0.0f, 0.0f, false};

    BufferedAsyncSerial::Ptr mSerialPort;
    std::thread mThread;

    Identifier mKey;

    bool started = false;

    uint16_t mExpectedLen;
    std::array<uint8_t, bufferLen> mPacketBuffer;
    size_t mPacketLen;
    std::array<uint8_t, headerLen> mHeaderBuffer;
    size_t mHeaderLen;
    bool mCheckingHeader;
    std::array<uint8_t, sendBufferLen> mSendBuffer;
    size_t mSendBufferLen;

    std::chrono::steady_clock::time_point lastSend;
    float testYaw = 0.0f, testPitch = 0.0f;
    bool testStart = false;

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

    void handlePacket(uint16_t id) {
        switch(id) {
            case(GimbalFdbPacket::id): {
                GimbalFdbPacket fdb(mPacketBuffer);
                //                std::cout << "yaw: " << fdb.yaw << " pitch: " << fdb.pitch << std::endl;
                //                HubLogger::watch("yaw", fdb.yaw);
                //                HubLogger::watch("pitch", fdb.pitch);
                fdb.yaw = (fdb.yaw < 0) ? fdb.yaw += 6.2831852 : fdb.yaw;

                // fdb.yaw = 0.0;// for standard

                const HeadInfo info{ SynchronizedClock::instance().now(),
                                     decltype(HeadInfo::transform){ glm::lookAtRH(
                                         glm::dvec3{ 0.0, mConfig.headHeightOffset, 0.0 },
                                         glm::dvec3{ std::cos(fdb.yaw + glm::half_pi<double>()) * std::cos(fdb.pitch),
                                                     mConfig.headHeightOffset + std::sin(fdb.pitch),
                                                     -std::sin(fdb.yaw + glm::half_pi<double>()) * std::cos(fdb.pitch) },
                                         glm::dvec3{ 0.0, 1.0, 0.0 }) },
                                     0.0, 0.0 };
                BlackBoard::instance().updateSync(mKey, info);
                PostureData posture;
                posture.lastUpdate = SynchronizedClock::instance().now();
                posture.postureOfRobot =
                    Transform<FrameOfReference::Ground, FrameOfReference::Robot>{ glm::identity<glm::dmat4>() };
                posture.angularAccelerationOfRobot =
                    Vector<UnitType::AngularAcceleration, FrameOfReference::Ground>{ glm::zero<glm::dvec3>() };
                posture.angularVelocityOfRobot =
                    Vector<UnitType::AngularVelocity, FrameOfReference::Ground>{ glm::zero<glm::dvec3>() };
                posture.linearAccelerationOfRobot =
                    Vector<UnitType::LinearAcceleration, FrameOfReference::Ground>{ glm::zero<glm::dvec3>() };
                posture.linearVelocityOfRobot =
                    Vector<UnitType::LinearVelocity, FrameOfReference::Ground>{ glm::zero<glm::dvec3>() };
                BlackBoard::instance().updateSync(mKey, posture);
                sendAll(update_posture_atom_v, mKey);
                sendAll(update_head_atom_v, mKey);
                break;
            }
        }
    }



    void sendPacket() {
        if(mSendBufferLen > sendBufferLen)
            mSendBufferLen = 0;
        if(mSendBufferLen == 0)
            return;
//        auto now = SynchronizedClock::instance().now();
//        std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(now - lastSend).count() << std::endl;
//        lastSend = now;
        mSerialPort->write(reinterpret_cast<char*>(mSendBuffer.data()), mSendBufferLen);
        mSendBufferLen = 0;
    }

public:
    SerialPort(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mSerialPort(std::make_unique<BufferedAsyncSerial>()), mKey{ typeid(SerialPort).hash_code() },
          mCheckingHeader(false) {
        const auto [devPath, baudRate, offset] = mConfig;
        mSerialPort->open(devPath, baudRate);
        mThread = std::thread{ [this]() {

            while(globalStatus == RunStatus::running) {
                receive();
                sendPacket();
                std::this_thread::sleep_for(0.5ms);
                memcpy(mSendBuffer.data() + mSendBufferLen, gimbalSetPacket.buffer.data(), GimbalSetPacket::size);
                mSendBufferLen += GimbalSetPacket::size;
            }


        } };
    }

    ~SerialPort() override {
        mSerialPort.release()->close();
        mThread.detach();
    }

    caf::behavior make_behavior() override {
        return { [this](start_atom) { started = true; },
                 [this](set_target_info_atom, double yawAngle, double pitchAngle, bool isFire) {
                     //
                     //#     identifier = "KE0200080465"
                     std::cout << yawAngle << " " << pitchAngle << std::endl;
//                     HubLogger::watch("targetYaw", yawAngle);
//                     HubLogger::watch("targetPitch", pitchAngle);
                     gimbalSetPacket = { static_cast<float>(yawAngle), static_cast<float>(pitchAngle), isFire };
                     //                     GimbalSetPacket gimbalSetPacket{ static_cast<float>(testYaw),
                     //                     static_cast<float>(testPitch), isFire };
                     //
                     //                     auto now = SynchronizedClock::instance().now();
                     //                     if ( static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(now -
                     //                     lastSend).count()) > 500) {
                     //                         lastSend = now;
                     //                         return;
                     //                     }
                     ////                     std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(now -
                     /// lastSend).count() * 0.001 <<std::endl;
                     //                     testYaw += std::chrono::duration_cast<std::chrono::milliseconds>(now -
                     //                     lastSend).count() * 0.001 * 0.1; testYaw = testYaw > glm::half_pi<float>() / 2.0f ?
                     //                     glm::half_pi<float>()/ 2.0f : testYaw; mSerialPort->write(reinterpret_cast<const
                     //                     char*>(gimbalSetPacket.buffer.data()), GimbalSetPacket::size);
//                     std::cout << "send "
//                               << "yaw: " << yawAngle << " pitch: " << pitchAngle << std::endl;
                     // lastSend = now;
                     //                     mSerialPort->write(reinterpret_cast<const char*>(gimbalSetPacket.buffer.data()),
                     //                     GimbalSetPacket::size); auto now = SynchronizedClock::instance().now(); std::clog <<
                     //                     std::chrono::duration_cast<std::chrono::milliseconds>( now - lastSend).count() <<
                     //                     std::endl; lastSend = now;
                 } };
    }
};

HUB_REGISTER_CLASS(SerialPort);
