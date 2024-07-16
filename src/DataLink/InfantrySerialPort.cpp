#include "AsyncSerial/BufferedAsyncSerial.h"
#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedEnergyFan.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"
#include "SerialPort/Crc.hpp"
#include "SerialPort/PacketHelper.hpp"
#include "SerialPort/SerialPort.hpp"
#include "Utility.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/gtc/matrix_transform.hpp>

class InfantryRecvPacket final {
public:
    static constexpr uint16_t id = 0x0A;

    float yaw, pitch, roll, bulletSpeed, speedX, speedY;
    uint8_t color, energyMode = 0, priorNum = 8 /*Negative*/;
    float capEnergy, chasisPower;
    explicit InfantryRecvPacket(std::array<uint8_t, 1024>& buffer) {
        PacketReader<1024> reader(buffer);
        yaw = reader.readCompressedFloat(-4.0f, 0.0005f);
        pitch = reader.readCompressedFloat(-4.0f, 0.0005f);
        roll = reader.readCompressedFloat(-4.0f, 0.0005f);
        speedX = reader.readCompressedFloat(-20.0f, 0.01f);
        speedY = reader.readCompressedFloat(-20.0f, 0.01f);
        const auto mask = reader.read();
        color = mask & 1;
        if(((mask >> 1) & 1) == 1) {
            energyMode = 1;
        } else if(((mask >> 2) & 1) == 1) {
            energyMode = 2;
        }
        priorNum = mask >> 3;
        bulletSpeed = reader.readCompressedFloat(-1.0f, 0.005f);
        capEnergy = reader.readCompressedFloat(-1.0f, 0.1f);
        chasisPower = reader.readCompressedFloat(-1.0f, 0.01f);
    }
};

class InfantrySendPacket final {
public:
    static constexpr uint16_t id = 0x0F;

    float yaw, pitch, horizontalDist, z;
    bool isFire;
    uint8_t hasTargets{}, targetType{};

    PacketBuffer<7, id> buffer{};

    void serialize() {
        buffer = {};
        buffer.serialize(yaw, -4.0f, 0.0005f);
        buffer.serialize(pitch, -4.0f, 0.0005f);
        buffer.serialize(horizontalDist, -4.0f, 0.0005f);
        buffer.serialize(static_cast<uint8_t>(static_cast<uint8_t>(isFire) | static_cast<uint8_t>(hasTargets << 1) |
                                              static_cast<uint8_t>(targetType << 2)));
        buffer.serializeCrc16();
    }
};

struct InfantrySerialPortSettings final {
    std::string devPath;
    uint32_t baudRate;
};

template <class Inspector>
bool inspect(Inspector& f, InfantrySerialPortSettings& x) {
    return f.object(x).fields(f.field("devPath", x.devPath), f.field("baudRate", x.baudRate));
}

class InfantrySerialPort final : public HubHelper<caf::event_based_actor, InfantrySerialPortSettings, update_head_atom,
                                                  update_posture_atom, energy_detector_control_atom>,
                                 public SerialPort<InfantryRecvPacket, InfantrySendPacket> {
    Identifier mKey;

    constexpr static size_t latencyLen = 100;
    std::deque<double> mLatency;

    float mYaw = 0., mPitch = 0., mRoll = 0., mCapEnergy = 0., mChasisPower = 0.;
    TimePoint mLastReceivedTime, mLastTargetTime;

    void infantryRecvCB(const InfantryRecvPacket& fdb) {
        GlobalSettings::get().bulletSpeed = fdb.bulletSpeed;
        HubLogger::watch("bullet speed", GlobalSettings::get().bulletSpeed);

        auto deltaYaw1 = mSendPacket.yaw - fdb.yaw;
        auto deltaPitch1 = mSendPacket.pitch - fdb.pitch;
        HubLogger::watch("deltaYaw1", glm::degrees(deltaYaw1));
        HubLogger::watch("deltaPitch1", glm::degrees(deltaPitch1));

        GlobalSettings::get().setColor(fdb.color == 0 ? Color::Red : Color::Blue);
        HubLogger::watch("selfColor", GlobalSettings::get().getColor() == Color::Red ? "Red" : "Blue");

        GlobalSettings::get().taskMode = fdb.energyMode;
        GlobalSettings::get().priorNum = fdb.priorNum;

        const double yaw = fdb.yaw;
        const double pitch = fdb.pitch;
        const double roll = fdb.roll;
        const HeadInfo infoHead{ SynchronizedClock::instance().now(), { roll, pitch, yaw } };

        mYaw = fdb.yaw;
        mPitch = fdb.pitch;
        mRoll = fdb.roll;
        mCapEnergy = fdb.capEnergy;
        mChasisPower = fdb.chasisPower;

        PostureData posture;
        posture.lastUpdate = SynchronizedClock::instance().now();
        posture.tfGround2Robot = Transform<FrameOfRef::Ground, FrameOfRef::Robot>{ glm::identity<glm::dmat4>() };
        posture.linearVelocityOfRobot = Vector<UnitType::LinearVelocity, FrameOfRef::Ground>{ { fdb.speedX, 0, -fdb.speedY } };

        GlobalSettings::get().gimbalYaw = mYaw;
        GlobalSettings::get().gimbalPitch = mPitch;
        GlobalSettings::get().gimbalRoll = mRoll;
        HubLogger::watch("gimbalYaw", GlobalSettings::get().gimbalYaw);
        HubLogger::watch("gimbalPitch", GlobalSettings::get().gimbalPitch);
        HubLogger::watch("gimbalRoll", GlobalSettings::get().gimbalRoll);
        sendAll(update_posture_atom_v, BlackBoard::instance().updateSync(mKey, posture));
        sendMasked(update_head_atom_v, 1U, 1U, BlackBoard::instance().updateSync(mKey, infoHead));
    }

    void infantrySetPacket() {
        mSendPacket.hasTargets = 0;
        if(Clock::now() - mLastTargetTime < 500ms) {
            mSendPacket.hasTargets |= 1;
        }
        HubLogger::watch("hasTargets", mSendPacket.hasTargets);
    }

public:
    InfantrySerialPort(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, std::move(name) }, SerialPort<InfantryRecvPacket, InfantrySendPacket>(
                                                          mConfig.devPath, mConfig.baudRate,
                                                          [this](auto&& PH1) {
                                                              infantryRecvCB(std::forward<decltype(PH1)>(PH1));
                                                          },
                                                          std::bind(&InfantrySerialPort::infantrySetPacket, this)),
          mKey{ generateKey(this) } {
        std::thread([this]() {
            while(globalStatus == RunStatus::running) {
                ReadableTimePoint readableTimePoint(std::chrono::system_clock::now());
                HubLogger::electricCtrlLog(
                    fmt::format("CapEnegy: {:.3f}, Chasis: {:.3f} Yaw: {:.3f}, Pitch: {:.3f}, Roll: {:.3f}", mCapEnergy,
                                mChasisPower, mYaw, mPitch, mRoll));
                std::this_thread::sleep_for(50ms);
            }
        }).detach();
    }
    caf::behavior make_behavior() override {
        return {
            [this](start_atom) {
                ACTOR_PROTOCOL_CHECK(start_atom);
                mStarted = true;
            },
            [this](set_target_info_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(set_target_info_atom, TypedIdentifier<SelectedTargetInfo>);
                auto data = BlackBoard::instance().get<SelectedTargetInfo>(key).value();
                double yawAngle = data.yawAngle;
                double pitchAngle = data.pitchAngle;
                glm::dvec3 targetPos = data.targetPos.has_value() ? data.targetPos.value() : glm::dvec3(0.0f, 0.0f, 0.0f);
                RobotType targetType = data.targetType.value();
                bool isFire = data.isFire;
                yawAngle = normalizeAngle(yawAngle - glm::half_pi<double>());

                {
                    std::lock_guard lock{ mPacketMutex };
                    mSendPacket.yaw = static_cast<float>(yawAngle);
                    mSendPacket.pitch = static_cast<float>(pitchAngle);
                    mSendPacket.isFire = isFire;
                    mSendPacket.horizontalDist = static_cast<float>(std::sqrt(square(targetPos.x) + square(targetPos.y)));
                    mSendPacket.z = targetPos.z;
                    mSendPacket.targetType = tfRobotType(targetType);
                    mLastTargetTime = Clock::now();
                }

                const auto current = Clock::now();
                const auto latency = static_cast<double>(current.time_since_epoch().count() - data.lastUpdate.time_since_epoch().count()) /
                    Duration::period::den * Duration::period::num;

                if(mLatency.size() >= latencyLen) {
                    mLatency.pop_front();
                }
                mLatency.push_back(latency);
                GlobalSettings::get().latency = avg(mLatency);
                HubLogger::watch("avgLatency", static_cast<int>(GlobalSettings::get().latency * 1000));
                HubLogger::watch("targetYaw1", yawAngle);
                HubLogger::watch("targetPitch1", pitchAngle);
                // HubLogger::watch("targetTypeReferee", tfRobotType(targetType));
                // HubLogger::watch("targetHorizontalDist", std::sqrt(square(targetPos.x) + square(targetPos.y)));
                HubLogger::visualLog(
                    fmt::format("InfantrySerialPort: target yaw: {:.3f}, target pitch: {:.3f},nowLatency: {}ms avgLatency: {}ms",
                                yawAngle, pitchAngle, static_cast<int>(mLatency.back() * 1000),
                                static_cast<int>(GlobalSettings::get().latency * 1000)));
            },
        };
    }
};

HUB_REGISTER_CLASS(InfantrySerialPort);