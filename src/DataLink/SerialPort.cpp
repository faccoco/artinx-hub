#include "AsyncSerial/BufferedAsyncSerial.h"
#include "BlackBoard.hpp"
#include "EnergyDetect.hpp"
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
    double headHeightOffset1;
    double headHeightOffset2;
    bool enableEnergyControl;
};

template <class Inspector>
bool inspect(Inspector& f, SerialPortSettings& x) {
    return f.object(x).fields(f.field("devPath", x.devPath), f.field("baudRate", x.baudRate),
                              f.field("headHeightOffset1", x.headHeightOffset1).fallback(0.0),
                              f.field("headHeightOffset2", x.headHeightOffset2).fallback(0.0),
                              f.field("enableEnergyControl", x.enableEnergyControl).fallback(false));
}

class SerialPort final : public HubHelper<caf::event_based_actor, SerialPortSettings, update_head_atom, update_posture_atom,
                                          energy_detector_control_atom> {
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

    void handlePacket(uint16_t id) {
        if(id == FdbPacket::id) {
            FdbPacket fdb(mPacketBuffer);
            if(fdb.bulletSpeed > 10.0f)
                GlobalSettings::get().bulletSpeed = fdb.bulletSpeed;
            HubLogger::watch("bullet speed", GlobalSettings::get().bulletSpeed);

            if(mConfig.enableEnergyControl) {
                HubLogger::watch("energy mode", static_cast<bool>(fdb.energyMode));
                sendAll(energy_detector_control_atom_v, static_cast<bool>(fdb.energyMode));
            }

            HubLogger::watch("yaw1", fdb.yaw);
            HubLogger::watch("pitch1", fdb.pitch);
            HubLogger::watch("yaw2", fdb.downYaw);
            HubLogger::watch("pitch2", fdb.downPitch);
            HubLogger::watch("speed x", fdb.speedX);
            HubLogger::watch("speed y", fdb.speedY);

            GlobalSettings::get().selfColor = (fdb.color == 0 ? Color::Red : Color::Blue);
            HubLogger::watch("self color", GlobalSettings::get().selfColor == Color::Red ? "Red" : "Blue");

            fdb.yaw = (fdb.yaw < 0.0f) ? fdb.yaw + glm::two_pi<float>() : fdb.yaw;
            fdb.downYaw = (fdb.downYaw < 0.0f) ? fdb.downYaw + glm::two_pi<float>() : fdb.downYaw;

            const HeadInfo infoUp{ SynchronizedClock::instance().now(),
                                   decltype(HeadInfo::transform){ glm::lookAtRH(
                                       glm::dvec3{ 0.0, mConfig.headHeightOffset1, 0.0 },
                                       glm::dvec3{ std::cos(fdb.yaw + glm::half_pi<double>()) * std::cos(fdb.pitch),
                                                   mConfig.headHeightOffset1 + std::sin(fdb.pitch),
                                                   -std::sin(fdb.yaw + glm::half_pi<double>()) * std::cos(fdb.pitch) },
                                       glm::dvec3{ 0.0, 1.0, 0.0 }) },
                                   0.0, 0.0 };
            const HeadInfo infoDown{ SynchronizedClock::instance().now(),
                                     decltype(HeadInfo::transform){ glm::lookAtRH(
                                         glm::dvec3{ 0.0, mConfig.headHeightOffset2, 0.0 },
                                         glm::dvec3{ std::cos(fdb.downYaw + glm::half_pi<double>()) * std::cos(fdb.downPitch),
                                                     mConfig.headHeightOffset2 + std::sin(fdb.downPitch),
                                                     -std::sin(fdb.downYaw + glm::half_pi<double>()) * std::cos(fdb.downPitch) },
                                         glm::dvec3{ 0.0, 1.0, 0.0 }) },
                                     0.0, 0.0 };

            PostureData posture;
            posture.lastUpdate = SynchronizedClock::instance().now();
            posture.postureOfRobot = Transform<FrameOfReference::Ground, FrameOfReference::Robot>{ glm::identity<glm::dmat4>() };
            posture.angularAccelerationOfRobot =
                Vector<UnitType::AngularAcceleration, FrameOfReference::Ground>{ glm::zero<glm::dvec3>() };
            posture.angularVelocityOfRobot =
                Vector<UnitType::AngularVelocity, FrameOfReference::Ground>{ glm::zero<glm::dvec3>() };
            const auto deltaTime = static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                                          lastReceivedTime - SynchronizedClock::instance().now())
                                                          .count()) /
                1000.0f;
            posture.linearAccelerationOfRobot =
                Vector<UnitType::LinearAcceleration, FrameOfReference::Ground>{ { (fdb.speedX - lastSpeedX) / deltaTime,
                                                                                  (fdb.speedY - lastSpeedY) / deltaTime, 0.0f } };
            lastReceivedTime = SynchronizedClock::instance().now();
            lastSpeedX = fdb.speedX;
            lastSpeedY = fdb.speedY;
            posture.linearVelocityOfRobot =
                Vector<UnitType::LinearVelocity, FrameOfReference::Ground>{ { fdb.speedX, fdb.speedY, 0.0f } };

            sendAll(update_posture_atom_v, BlackBoard::instance().updateSync(mKey, posture));
            sendMasked(update_head_atom_v, 1U, 1U, BlackBoard::instance().updateSync(mKey, infoUp));
            sendMasked(update_head_atom_v, 2U, 2U,
                       BlackBoard::instance().updateSync(Identifier{ mKey.val ^ 0xffffffff }, infoDown));
        }
    }

    void sendPacket() {
        if(mSendBufferLen > sendBufferLen)
            mSendBufferLen = 0;
        if(mSendBufferLen == 0)
            return;
        // std::cout << mSendBufferLen << std::endl;
        mSerialPort->write(reinterpret_cast<char*>(mSendBuffer.data()), mSendBufferLen);
        mSendBufferLen = 0;
    }

    float mYaw1 = 0.0f, mPitch1 = 0.0f, mYaw2 = 0.0f, mPitch2 = 0.0f;
    bool mFire1 = false, mFire2 = false;

public:
    SerialPort(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mSerialPort(std::make_unique<BufferedAsyncSerial>()), mKey{ generateKey(this) },
          mCheckingHeader(false) {
        mSerialPort->open(mConfig.devPath, mConfig.baudRate);
        lastReceivedTime = SynchronizedClock::instance().now();
        mThread = std::thread{ [this]() {
            while(globalStatus == RunStatus::running) {
                receive();
                sendPacket();
                std::this_thread::sleep_for(1.5ms);
//                int targetBits = 0;
//                if(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - lastUpTargetTime).count() < 500) {
//                    targetBits |= (1 << 2);
//                }
//                if(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - lastDownTargetTime).count() < 500) {
//                    targetBits |= (1 << 3);
//                }
//                if (targetBits) {
//                    gimbalSetPacket.buffer.now -= 3;
//                    gimbalSetPacket.buffer.serialize(gimbalSetPacket.buffer.buffer[gimbalSetPacket.buffer.now] | targetBits);
//                    gimbalSetPacket.buffer.serializeCrc16();
//                }
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
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                    started = true;
                },
                 [this](set_target_info_atom, GroupMask mask, Clock::rep begin, double yawAngle, double pitchAngle, bool isFire) {
                     ACTOR_PROTOCOL_CHECK(set_target_info_atom, GroupMask, Clock::rep, double, double, bool);

                     const auto current = Clock::now();

                     HubLogger::watch("latency", (current.time_since_epoch().count() - begin) / 1'000'000);
                     HubLogger::watch(
                         fmt::format("target yaw{}", mask),
                         fmt::format("{:.8f}", yawAngle > glm::pi<double>() ? (yawAngle - glm::two_pi<double>()) : yawAngle));
                     HubLogger::watch(fmt::format("target pitch{}", mask), fmt::format("{:.8f}", pitchAngle));

                     if(mask == 1U) {
                         mYaw1 = static_cast<float>(yawAngle);
                         mPitch1 = static_cast<float>(pitchAngle);
                         mFire1 = isFire;
                         lastUpTargetTime = current;
                     } else {
                         mYaw2 = static_cast<float>(yawAngle);
                         mPitch2 = static_cast<float>(pitchAngle);
                         mFire2 = isFire;
                         lastDownTargetTime = current;
                     }

                     gimbalSetPacket = GimbalSetPacket(mYaw1, mPitch1, mFire1, mYaw2, mPitch2, mFire2);
                 } };
    }
};

HUB_REGISTER_CLASS(SerialPort);
