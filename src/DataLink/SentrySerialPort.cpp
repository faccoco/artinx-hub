#include "AsyncSerial/BufferedAsyncSerial.h"
#include "BlackBoard.hpp"
#include "EnergyDetect.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"
#include "SerialPort/PacketHelper.hpp"
#include "SerialPort/SerialPort.hpp"
#include "Utility.hpp"

#include <algorithm>
#include <iterator>

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/gtc/matrix_transform.hpp>

class SentryRecvPacket final {
public:
    static constexpr uint16_t id = 0xE0;

    float yaw, pitch, bulletSpeed, speedX, speedY;
    uint8_t color, priorNum;
    float capEnergy, chasisPower;
    bool blockSentry, blockEngineer;
    struct ElectricData {
        float bulletSpeed30Offset, fdbPositionX, fdbPositionY, fdbYawInWorld, fricLeftRpm, fricRightRpm, heat;
    } electricData;
    explicit SentryRecvPacket(std::array<uint8_t, 1024>& buffer) {
        PacketReader<1024> reader(buffer);
        const auto mask = reader.read();
        color = mask & 1;
        priorNum = (mask >> 1) & 0x7;
        blockEngineer = (mask >> 4) & 1;
        blockSentry = (mask >> 5) & 1;
        yaw = reader.readCompressedFloat(-4.0f, 0.0005f);
        pitch = reader.readCompressedFloat(-4.0f, 0.0005f);
        bulletSpeed = reader.readCompressedFloat(-1.0f, 0.005f);
        electricData.bulletSpeed30Offset = reader.readCompressedFloat(-100.0f, 0.1f);
        electricData.fricLeftRpm = reader.readCompressedFloat(0.0f, 1.0f);
        electricData.fricRightRpm = reader.readCompressedFloat(0.0f, 1.0f);
        electricData.heat = reader.readCompressedFloat(0.0f, 0.1f);

        electricData.fdbPositionX = reader.readCompressedFloat(-1.0f, 0.001f);
        electricData.fdbPositionY = reader.readCompressedFloat(-1.0f, 0.001f);
        electricData.fdbYawInWorld = reader.readCompressedFloat(-1.0f, 0.01f);
    }
};

class SentrySendPacket final {
public:
    static constexpr uint16_t id = 0xB0;

    float yaw, pitch;
    bool isFire;
    uint8_t hasTargets{};

    PacketBuffer<5, id> buffer{};

    void serialize() {
        buffer = {};
        buffer.serialize(hasTargets);
        buffer.serialize(yaw, -4.0f, 0.0005f);
        buffer.serialize(pitch, -4.0f, 0.0005f);
        buffer.serializeCrc16();
    }
};

struct SentrySerialPortSettings final {
    std::string devPath;
    uint32_t baudRate;
};

template <class Inspector>
bool inspect(Inspector& f, SentrySerialPortSettings& x) {
    return f.object(x).fields(f.field("devPath", x.devPath), f.field("baudRate", x.baudRate));
}

class SentrySerialPort final
    : public HubHelper<caf::event_based_actor, SentrySerialPortSettings, update_head_atom, update_posture_atom>,
      public SerialPort<SentryRecvPacket, SentrySendPacket> {
    Identifier mKey;

    constexpr static size_t latencyLen = 100;
    std::deque<double> mLatency;

    TimePoint mLastReceivedTime, mLastTargetTime;

    SentryRecvPacket::ElectricData mElectricDataBuff;

    void infantryRecvCB(const SentryRecvPacket& fdb) {
        if(fdb.bulletSpeed > 15)
            GlobalSettings::get().bulletSpeed = fdb.bulletSpeed;
        HubLogger::watch("bullet speed", GlobalSettings::get().bulletSpeed);

        auto deltaYaw1 = mSendPacket.yaw - fdb.yaw;
        auto deltaPitch1 = mSendPacket.pitch - fdb.pitch;
        HubLogger::watch("deltaYaw1", deltaYaw1);
        HubLogger::watch("deltaPitch1", deltaPitch1);
        HubLogger::watch("yaw", fdb.yaw);
        HubLogger::watch("pitch", fdb.pitch);

        GlobalSettings::get().setColor(fdb.color == 0 ? Color::Red : Color::Blue);
        HubLogger::watch("selfColor", GlobalSettings::get().getColor() == Color::Red ? "Red" : "Blue");

        GlobalSettings::get().priorNum = fdb.priorNum;
        GlobalSettings::get().blockEngineer = fdb.blockEngineer;
        GlobalSettings::get().blockSentry = fdb.blockSentry;

        const HeadInfo infoHead{ SynchronizedClock::instance().now(), { 0.0, fdb.pitch, fdb.yaw } };

        PostureData posture;
        posture.lastUpdate = SynchronizedClock::instance().now();
        posture.tfGround2Robot = Transform<FrameOfRef::Ground, FrameOfRef::Robot>{ glm::identity<glm::dmat4>() };
        posture.linearVelocityOfRobot = Vector<UnitType::LinearVelocity, FrameOfRef::Ground>{ { fdb.speedX, 0, -fdb.speedY } };

        sendAll(update_posture_atom_v, BlackBoard::instance().updateSync(mKey, posture));
        sendMasked(update_head_atom_v, 1U, 1U, BlackBoard::instance().updateSync(mKey, infoHead));

        mElectricDataBuff = fdb.electricData;
    }

    void sentrySetPacket() {
        mSendPacket.hasTargets = 0;
        if(Clock::now() - mLastTargetTime < 500ms)
            mSendPacket.hasTargets |= 1;
        HubLogger::watch("hasTargets", mSendPacket.hasTargets);
    }

public:
    SentrySerialPort(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, std::move(name) },
          SerialPort<SentryRecvPacket, SentrySendPacket>(
              mConfig.devPath, mConfig.baudRate, std::bind(&SentrySerialPort::infantryRecvCB, this, std::placeholders::_1),
              std::bind(&SentrySerialPort::sentrySetPacket, this)),
          mKey{ generateKey(this) } {
        std::thread([this]() {
            while(globalStatus == RunStatus::running) {
                ReadableTimePoint readableTimePoint(std::chrono::system_clock::now());
                HubLogger::electricCtrlLog(
                    fmt::format("{}:{}:{} bulletSpeed30_offset:{:.3f} fdb_position_x:{:.3f} fdb_position_y:{:.3f} "
                                "fdb_yaw_in_world:{:.3f} fric_left_rpm:{} fric_right_rpm:{} heat:{}",
                                readableTimePoint.tm.tm_hour, readableTimePoint.tm.tm_min, readableTimePoint.tm.tm_sec,
                                mElectricDataBuff.bulletSpeed30Offset, mElectricDataBuff.fdbPositionX,
                                mElectricDataBuff.fdbPositionY, mElectricDataBuff.fdbYawInWorld, mElectricDataBuff.fricLeftRpm,
                                mElectricDataBuff.fricRightRpm, mElectricDataBuff.heat));
                std::this_thread::sleep_for(10ms);
            }
        }).detach();
    }

    caf::behavior make_behavior() override {
        return {
            [this](start_atom) {
                ACTOR_PROTOCOL_CHECK(start_atom);
                started = true;
            },
            [this](set_target_info_atom, GroupMask mask, Clock::rep begin, double yawAngle, double pitchAngle, bool isFire,
                   SolverType solverType) {
                ACTOR_PROTOCOL_CHECK(set_target_info_atom, GroupMask, Clock::rep, double, double, bool, SolverType);

                yawAngle = normalizeAngle(yawAngle - glm::half_pi<double>());
                HubLogger::watch("targetYaw", yawAngle);
                HubLogger::watch("targetPitch", pitchAngle);

                {
                    std::lock_guard lock{ mPacketMutex };
                    mSendPacket.yaw = static_cast<float>(yawAngle);
                    mSendPacket.pitch = static_cast<float>(pitchAngle);
                    mSendPacket.isFire = isFire;
                    mLastTargetTime = Clock::now();
                }

                const auto current = Clock::now();
                const auto latency =
                    double(current.time_since_epoch().count() - begin) / Duration::period::den * Duration::period::num;

                if(mLatency.size() >= latencyLen)
                    mLatency.pop_front();
                mLatency.push_back(latency);
                GlobalSettings::get().latency = avg(mLatency);
                HubLogger::watch("avgLatency", static_cast<int>(GlobalSettings::get().latency * 1000));
                HubLogger::visualLog(
                    fmt::format("SentrySerialPort: target yaw: {:.3f}, target pitch: {:.3f},nowLatency: {}ms avgLatency: {}ms",
                                yawAngle, pitchAngle, static_cast<int>(mLatency.back() * 1000),
                                static_cast<int>(GlobalSettings::get().latency * 1000)));
            },
        };
    }
};

HUB_REGISTER_CLASS(SentrySerialPort);