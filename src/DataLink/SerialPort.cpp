#include "BlackBoard.hpp"
#include "EnergyDetect.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "Packet.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include "AsyncSerial/BufferedAsyncSerial.h"
#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/gtc/matrix_transform.hpp>

#include "SuppressWarningEnd.hpp"

struct SerialPortSettings final {
    std::string devPath;
    uint32_t baudRate;
    double headHeightOffset1;
    double headHeightOffset2;
    double headForwardOffset1;
    double headForwardOffset2;
    bool enableEnergyControl;
    double minBulletSpeed;
    double maxBulletSpeed;
    bool getBulletSpeedFromSerial;
};

template <class Inspector>
bool inspect(Inspector& f, SerialPortSettings& x) {
    return f.object(x).fields(f.field("devPath", x.devPath), f.field("baudRate", x.baudRate),
                              f.field("headHeightOffset1", x.headHeightOffset1).fallback(0.0),
                              f.field("headHeightOffset2", x.headHeightOffset2).fallback(0.0),
                              f.field("headForwardOffset1", x.headForwardOffset1).fallback(0.0),
                              f.field("headForwardOffset2", x.headForwardOffset2).fallback(0.0),
                              f.field("enableEnergyControl", x.enableEnergyControl).fallback(false),
                              f.field("minBulletSpeed", x.minBulletSpeed).fallback(0.0),
                              f.field("maxBulletSpeed", x.maxBulletSpeed).fallback(100.0),
                              f.field("getBulletSpeedFromSerial", x.getBulletSpeedFromSerial).fallback(false));
}

class SerialPort final : public HubHelper<caf::event_based_actor, SerialPortSettings, update_head_atom, update_posture_atom,
                                          energy_detector_control_atom, outpost_detector_control_atom> {
    constexpr static size_t bufferLen = 1024;
    constexpr static size_t headerLen = 5;
    constexpr static size_t sendBufferLen = 1024;
    constexpr static size_t latencyLen = 100;
    constexpr static size_t mShootDelayLen = 5;
    constexpr static std::uint16_t maxShootDelay = 500;
    constexpr static size_t mBulletSpeedLen = 5;

    constexpr static Duration ChassisPowerRecordInterval = 100ms;

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

    float mLastSpeedX = 0.0f, mLastSpeedY = 0.0f;
    TimePoint mLastReceivedTime, mLastUpTargetTime, mLastDownTargetTime;

    std::atomic<float> mCapEnergy, mChasisPower;

    bool mOutpostMode = false;
    std::mutex mOutpostModeChangeMutex;

    std::deque<double> mLatency;
    std::deque<uint16_t> mShootDelay;
    std::deque<double> mBulletSpeed;
    std::optional<double> mLastBulletSpeed;

    bool mHaveReceivedFdbPacket = false;

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

    void handlePacket(uint16_t id) {
        if(id == FdbPacket::id) {
            reportFrameRate(Clock::now());

            FdbPacket fdb(mPacketBuffer);
            mHaveReceivedFdbPacket = true;
            /*            if((!mShootDelay.empty()) && (fdb.shootDelayTime != mShootDelay.back()))
                            logInfo(fmt::format("shoot delay {}", fdb.shootDelayTime));*/
            if(mConfig.getBulletSpeedFromSerial) {
                if(!mLastBulletSpeed.has_value() || mLastBulletSpeed.value() != fdb.bulletSpeed) {
                    HubLogger::ElectricCtrlLog(fmt::format("bulletSpeed: {}", fdb.bulletSpeed));
                    if(fdb.bulletSpeed > mConfig.minBulletSpeed && fdb.bulletSpeed < mConfig.maxBulletSpeed) {
                        if(mBulletSpeed.size() >= mBulletSpeedLen)
                            mBulletSpeed.pop_front();
                        mBulletSpeed.push_back(fdb.bulletSpeed);
                        if(mBulletSpeed.size() < 3) {
                            GlobalSettings::get().bulletSpeed = avg(mBulletSpeed);
                        } else {
                            double maxSpeed = mBulletSpeed[0], minSpeed = mBulletSpeed[0], sumSpeed = 0;
                            for(auto speed : mBulletSpeed) {
                                if(speed > maxSpeed)
                                    maxSpeed = speed;
                                if(speed < minSpeed)
                                    minSpeed = speed;
                                sumSpeed += speed;
                            }
                            GlobalSettings::get().bulletSpeed = (sumSpeed - maxSpeed - minSpeed) / (mBulletSpeed.size() - 2);
                        }
                        HubLogger::watch("bulletSpeed", GlobalSettings::get().bulletSpeed);
                    }
                    mLastBulletSpeed = fdb.bulletSpeed;
                }
            }
            if(fdb.shootDelayTime < maxShootDelay && (mShootDelay.empty() || fdb.shootDelayTime != mShootDelay.back())) {
                if(mShootDelay.size() >= mShootDelayLen)
                    mShootDelay.pop_front();
                mShootDelay.push_back(fdb.shootDelayTime);
                GlobalSettings::get().shootDelayTime = avg(mShootDelay) / 1000.0;  // ms -> s
            }
            HubLogger::watch("bullet speed", GlobalSettings::get().bulletSpeed);
            // HubLogger::watch("fdb bullet speed", fdb.bulletSpeed);
            // HubLogger::watch("fdb shoot delay time", fdb.shootDelayTime);
            // HubLogger::watch("shoot delay time", static_cast<int>(GlobalSettings::get().shootDelayTime * 1000));

            if(mConfig.enableEnergyControl) {
                // HubLogger::watch("energy mode", static_cast<bool>(fdb.energyMode));
                sendAll(energy_detector_control_atom_v, static_cast<bool>(fdb.energyMode));
            }

            auto deltaYaw1 = gimbalSetPacket.up.yaw - fdb.yaw;
            auto deltaPitch1 = gimbalSetPacket.up.pitch - fdb.pitch;
            HubLogger::watch("yaw1", fdb.yaw);
            HubLogger::watch("pitch1", fdb.pitch);
            HubLogger::watch("roll1", fdb.roll);
            // HubLogger::watch("yaw2", fdb.downYaw);
            // HubLogger::watch("pitch2", fdb.downPitch);
            // HubLogger::watch("speed x", fdb.speedX);
            // HubLogger::watch("speed y", fdb.speedY);
            HubLogger::watch("deltaYaw1", deltaYaw1);
            HubLogger::watch("deltaPitch1", deltaPitch1);
            //            logInfo(fmt::format("{:.5f} {:.5f} {:.5f}",fdb.yaw,fdb.pitch,fdb.roll));

            GlobalSettings::get().setColor(fdb.color == 0 ? Color::Red : Color::Blue);
            HubLogger::watch("selfColor", GlobalSettings::get().getColor() == Color::Red ? "Red" : "Blue");

            fdb.yaw = (fdb.yaw < 0.0f) ? fdb.yaw + glm::two_pi<float>() : fdb.yaw;
            fdb.downYaw = (fdb.downYaw < 0.0f) ? fdb.downYaw + glm::two_pi<float>() : fdb.downYaw;

            if(!mOutpostMode && fdb.outpostMode) {
                std::lock_guard lock{ mOutpostModeChangeMutex };
                mOutpostMode = true;
                gimbalSetPacket.setUpTarget(fdb.yaw, fdb.pitch, false);
            }
            mOutpostMode = fdb.outpostMode;
            sendAll(outpost_detector_control_atom_v, static_cast<bool>(mOutpostMode));
            HubLogger::watch("outpost mode", static_cast<bool>(mOutpostMode));
            mCapEnergy = fdb.capEnergy;
            mChasisPower = fdb.chasisPower;

            const double yaw = -fdb.yaw - glm::half_pi<double>();
            const double pitch = fdb.pitch;
            const double roll = fdb.roll;
            const HeadInfo infoUp{ SynchronizedClock::instance().now(),
                                   decltype(HeadInfo::tfRobot2Gun){ glm::lookAtRH(
                                       glm::dvec3{ 0.0, mConfig.headHeightOffset1, mConfig.headForwardOffset1 },
                                       glm::dvec3{ std::cos(pitch) * std::cos(yaw), mConfig.headHeightOffset1 + std::sin(pitch),
                                                   mConfig.headForwardOffset1 + std::cos(pitch) * std::sin(yaw) },
                                       glm::dvec3{ sin(roll), cos(roll), 0.0 }) } };
            const double downYaw = -fdb.downYaw - glm::half_pi<double>();
            const double downPitch = fdb.downPitch;
            const HeadInfo infoDown{ SynchronizedClock::instance().now(),
                                     decltype(HeadInfo::tfRobot2Gun){ glm::lookAtRH(
                                         glm::dvec3{ 0.0, mConfig.headHeightOffset1, mConfig.headForwardOffset1 },
                                         glm::dvec3{ std::cos(downPitch) * std::cos(downYaw),
                                                     mConfig.headHeightOffset1 + std::sin(downPitch),
                                                     mConfig.headForwardOffset1 + std::cos(downPitch) * std::sin(downYaw) },
                                         glm::dvec3{ 0.0, 1.0, 0.0 }) } };

            PostureData posture;
            posture.lastUpdate = SynchronizedClock::instance().now();
            posture.tfGround2Robot = Transform<FrameOfRef::Ground, FrameOfRef::Robot>{ glm::identity<glm::dmat4>() };
            mLastReceivedTime = SynchronizedClock::instance().now();
            mLastSpeedX = fdb.speedX;
            mLastSpeedY = fdb.speedY;
            posture.linearVelocityOfRobot =
                Vector<UnitType::LinearVelocity, FrameOfRef::Ground>{ { fdb.speedX, 0, -fdb.speedY } };

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
        mSerialPort->write(reinterpret_cast<char*>(mSendBuffer.data()), mSendBufferLen);
        mSendBufferLen = 0;
        HubLogger::watch("targetYaw1", gimbalSetPacket.up.yaw);
        HubLogger::watch("targetPitch1", gimbalSetPacket.up.pitch);
    }

public:
    SerialPort(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mSerialPort(std::make_unique<BufferedAsyncSerial>()), mKey{ generateKey(this) },
          mCheckingHeader(false), mSendBufferLen(0) {
        mSerialPort->open(mConfig.devPath, mConfig.baudRate);
        mLastReceivedTime = SynchronizedClock::instance().now();
        gimbalSetPacket.serialize();
        mThread = std::thread{ [this]() {
            while(globalStatus == RunStatus::running) {
                receive();
                static int sendTimes = 0;
                if(gimbalSetPacket.up.isFire && mOutpostMode && sendTimes == 0) {
                    sendTimes = 150;
                    //                    logInfo("start send fire");
                }
                if(sendTimes && !(--sendTimes))
                    gimbalSetPacket.up.isFire = false;
                std::this_thread::sleep_for(0.75ms);
                uint8_t targetBits = 0;
                if(mOutpostMode)
                    targetBits |= 1;
                else {
                    if(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - mLastUpTargetTime).count() < 500)
                        targetBits |= 1;
                    if(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - mLastDownTargetTime).count() < 500)
                        targetBits |= 2;
                }
                HubLogger::watch("hasTargets", targetBits);
                gimbalSetPacket.setHasTargetBits(targetBits);
                gimbalSetPacket.serialize();
                gimbalSetPacket.buffer.copyToSendBuffer(mSendBuffer.data() + mSendBufferLen);
                mSendBufferLen += gimbalSetPacket.buffer.size();
                sendPacket();
            }
        } };
        std::thread([this]() {
            while(globalStatus == RunStatus::running) {
                if(mHaveReceivedFdbPacket) {
                    HubLogger::ElectricCtrlLog(fmt::format("capEnergy: {:.1f} chasisPower: {:.2f}", mCapEnergy, mChasisPower));
                }
                std::this_thread::sleep_for(ChassisPowerRecordInterval);
            }
        }).detach();
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
                 [this](set_target_info_atom, GroupMask mask, Clock::rep begin, double yawAngle, double pitchAngle, bool isFire,
                        SolverType solverType) {
                     ACTOR_PROTOCOL_CHECK(set_target_info_atom, GroupMask, Clock::rep, double, double, bool, SolverType);

                     isFire = solverType && isFire;
                     {
                         std::lock_guard lock{ mOutpostModeChangeMutex };
                         if(mOutpostMode && solverType == normalSolver)
                             return;

                         yawAngle = normalizeAngle(yawAngle - glm::half_pi<double>());
                         if(mask == 1U) {
                             gimbalSetPacket.setUpTarget(static_cast<float>(yawAngle), static_cast<float>(pitchAngle), isFire);
                             mLastUpTargetTime = Clock::now();
                         } else {
                             gimbalSetPacket.setDownTarget(static_cast<float>(yawAngle), static_cast<float>(pitchAngle), isFire);
                             mLastDownTargetTime = Clock::now();
                         }
                     }

                     const auto current = Clock::now();
                     const auto latency =
                         double(current.time_since_epoch().count() - begin) / Duration::period::den * Duration::period::num;

                     if(mLatency.size() >= latencyLen)
                         mLatency.pop_front();
                     mLatency.push_back(latency);
                     GlobalSettings::get().latency = avg(mLatency);
                     HubLogger::watch("avgLatency", static_cast<int>(GlobalSettings::get().latency * 1000));
                     HubLogger::VisualLog(
                         fmt::format("SerialPort: target yaw: {:.3f}, target pitch: {:.3f}, avgLatency: {:.3f}ms", yawAngle,
                                     pitchAngle, GlobalSettings::get().latency * 1000));
                 } };
    }
};

HUB_REGISTER_CLASS(SerialPort);
