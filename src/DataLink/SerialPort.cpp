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

    GimbalSetPacket gimbalSetPacket{ 0.0f, 0.0f, false };

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

    float lastSpeedX = 0.0f, lastSpeedY = 0.0f;
    TimePoint lastReceivedTime;

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
        if(id == FdbPacket::id) {
            GlobalSettings::get().bulletSpeed = 13.0f;

            FdbPacket fdb(mPacketBuffer);

            HubLogger::watch("yaw", fdb.yaw);
            HubLogger::watch("pitch", fdb.pitch);
            //            HubLogger::watch("yaw2", fdb.downYaw);
            //            HubLogger::watch("pitch2", fdb.downPitch);
            HubLogger::watch("speed x", fdb.speedX);
            HubLogger::watch("speed y", fdb.speedY);
            //            HubLogger::watch("shoot spd", fdb.bulletSpeed);
            //            HubLogger::watch("color", fdb.color);
            //            HubLogger::watch("shooter", fdb.shooterId);

            GlobalSettings::get().selfColor = (fdb.color == 0 ? Color::Red : Color::Blue);
            HubLogger::watch("self color", GlobalSettings::get().selfColor == Color::Red ? "Red" : "Blue");

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
            posture.postureOfRobot = Transform<FrameOfReference::Ground, FrameOfReference::Robot>{ glm::identity<glm::dmat4>() };
            posture.angularAccelerationOfRobot =
                Vector<UnitType::AngularAcceleration, FrameOfReference::Ground>{ glm::zero<glm::dvec3>() };
            posture.angularVelocityOfRobot =
                Vector<UnitType::AngularVelocity, FrameOfReference::Ground>{ glm::zero<glm::dvec3>() };
            float deltaTime = static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                                     lastReceivedTime - SynchronizedClock::instance().now())
                                                     .count()) /
                1000.0f;
            posture.linearAccelerationOfRobot =
                Vector<UnitType::LinearAcceleration, FrameOfReference::Ground>{ { (fdb.speedX - lastSpeedX) / deltaTime,
                                                                                  (fdb.speedY - lastSpeedY) / deltaTime, 0.0f } };
            HubLogger::watch("acc", (fdb.speedX - lastSpeedX) / std::max(1e-6f, deltaTime));
            lastReceivedTime = SynchronizedClock::instance().now();
            lastSpeedX = fdb.speedX;
            lastSpeedY = fdb.speedY;
            posture.linearVelocityOfRobot =
                Vector<UnitType::LinearVelocity, FrameOfReference::Ground>{ { fdb.speedX, fdb.speedY, 0.0f } };
            BlackBoard::instance().updateSync(mKey, posture);
            sendAll(update_posture_atom_v, mKey);
            sendAll(update_head_atom_v, mKey);
        }
    }

    void sendPacket() {
        if(mSendBufferLen > sendBufferLen)
            mSendBufferLen = 0;
        if(mSendBufferLen == 0)
            return;
        //        for (int i = 0; i < mSendBufferLen; i++) {
        //            std::cout << std::hex << static_cast<int>(mSendBuffer[i]) << " ";
        //        }
        //        std::cout << std::endl;
        mSerialPort->write(reinterpret_cast<char*>(mSendBuffer.data()), mSendBufferLen);
        mSendBufferLen = 0;
    }

public:
    SerialPort(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mSerialPort(std::make_unique<BufferedAsyncSerial>()), mKey{ typeid(SerialPort).hash_code() },
          mCheckingHeader(false) {
        const auto [devPath, baudRate, offset] = mConfig;
        mSerialPort->open(devPath, baudRate);
        lastReceivedTime = SynchronizedClock::instance().now();
        mThread = std::thread{ [this]() {
            while(globalStatus == RunStatus::running) {
                receive();
                sendPacket();
                std::this_thread::sleep_for(0.75ms);
                gimbalSetPacket.buffer.copyToSendBuffer(mSendBuffer.data() + mSendBufferLen);
                mSendBufferLen += gimbalSetPacket.buffer.size();
            }
        } };
    }

    ~SerialPort() override {
        mSerialPort.release()->close();
        mThread.detach();
    }

    caf::behavior make_behavior() override {
        return { [this](start_atom) { started = true; },
                 [this](set_target_info_atom, Clock::rep begin, double yawAngle, double pitchAngle, bool isFire) {
                     const auto current = Clock::now();
                     HubLogger::watch("latency", (current.time_since_epoch().count() - begin) / 1'000'000);
                     HubLogger::watch("tgtyaw", yawAngle > glm::pi<double>() ? (yawAngle - glm::two_pi<double>()) : yawAngle);
                     HubLogger::watch("tgtpitch", pitchAngle);

                     gimbalSetPacket = GimbalSetPacket(static_cast<float>(yawAngle), static_cast<float>(pitchAngle), isFire);
                 } };
    }
};

HUB_REGISTER_CLASS(SerialPort);
