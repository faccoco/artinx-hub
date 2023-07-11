#include "AsyncSerial/BufferedAsyncSerial.h"
#include "BlackBoard.hpp"
#include "EnergyDetect.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"
#include "SerialPort/Crc.hpp"
#include "SerialPort/PacketHelper.hpp"
#include "SerialPort/SerialPort.hpp"
#include "Utility.hpp"

#include <algorithm>
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

    float yaw, pitch,  bulletSpeed, speedX, speedY;
    uint8_t color, energyMode;
    float capEnergy, chasisPower;
    explicit InfantryRecvPacket(std::array<uint8_t, 1024>& buffer) {
        PacketReader<1024> reader(buffer);
        yaw = reader.readCompressedFloat(-4.0f, 0.0005f);
        pitch = reader.readCompressedFloat(-4.0f, 0.0005f);
        speedX = reader.readCompressedFloat(-20.0f, 0.01f);
        speedY = reader.readCompressedFloat(-20.0f, 0.01f);
        const auto mask = reader.read();
        color = mask & 1;
        energyMode = (mask >> 1) & 1;
        bulletSpeed = reader.readCompressedFloat(-1.0f, 0.005f);
        capEnergy = reader.readCompressedFloat(-1.0f, 0.1f);
        chasisPower = reader.readCompressedFloat(-1.0f, 0.01f);
    }
};

class InfantrySendPacket final {
public:
    static constexpr uint16_t id = 0x0F;

    float yaw, pitch;
    bool isFire;
    uint8_t hasTargets{};

    PacketBuffer<5, id> buffer{};

    void serialize() {
        buffer = {};
        buffer.serialize(yaw, -4.0f, 0.0005f);
        buffer.serialize(pitch, -4.0f, 0.0005f);
        buffer.serialize(static_cast<uint8_t>(static_cast<uint8_t>(isFire) | (hasTargets << 1)));
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

    TimePoint mLastReceivedTime, mLastTargetTime;

    void infantryRecvCB(const InfantryRecvPacket& fdb) {
        GlobalSettings::get().bulletSpeed = fdb.bulletSpeed;
        HubLogger::watch("bullet speed", GlobalSettings::get().bulletSpeed);

        sendAll(energy_detector_control_atom_v, static_cast<bool>(fdb.energyMode));

        auto deltaYaw1 = mSendPacket.yaw - fdb.yaw;
        auto deltaPitch1 = mSendPacket.pitch - fdb.pitch;
        // HubLogger::watch("yaw1", fdb.yaw);
        // HubLogger::watch("pitch1", fdb.pitch);
        HubLogger::watch("deltaYaw1", deltaYaw1);
        HubLogger::watch("deltaPitch1", deltaPitch1);

        GlobalSettings::get().setColor(fdb.color == 0 ? Color::Red : Color::Blue);
        HubLogger::watch("selfColor", GlobalSettings::get().getColor() == Color::Red ? "Red" : "Blue");

        const double yaw = fdb.yaw + glm::half_pi<double>();
        const double pitch = fdb.pitch;
        const double roll = 0.0;
        const HeadInfo infoHead{ SynchronizedClock::instance().now(),
                                 {roll, pitch, yaw}};

        PostureData posture;
        posture.lastUpdate = SynchronizedClock::instance().now();
        posture.tfGround2Robot = Transform<FrameOfRef::Ground, FrameOfRef::Robot>{ glm::identity<glm::dmat4>() };
        posture.linearVelocityOfRobot = Vector<UnitType::LinearVelocity, FrameOfRef::Ground>{ { fdb.speedX, 0, -fdb.speedY } };

        sendAll(update_posture_atom_v, BlackBoard::instance().updateSync(mKey, posture));
        sendMasked(update_head_atom_v, 1U, 1U, BlackBoard::instance().updateSync(mKey, infoHead));
    }

    void infantrySetPacket() {
        mSendPacket.hasTargets = 0;
        if(Clock::now() - mLastTargetTime < 500ms)
            mSendPacket.hasTargets |= 1;
        HubLogger::watch("hasTargets", mSendPacket.hasTargets);
    }

public:
    InfantrySerialPort(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, SerialPort<InfantryRecvPacket, InfantrySendPacket>(
                                         mConfig.devPath, mConfig.baudRate,
                                         std::bind(&InfantrySerialPort::infantryRecvCB, this, std::placeholders::_1),
                                         std::bind(&InfantrySerialPort::infantrySetPacket, this)),
          mKey{ generateKey(this) } {}

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
                HubLogger::watch("targetYaw1", yawAngle);
                HubLogger::watch("targetPitch1", pitchAngle);
                HubLogger::visualLog(
                    fmt::format("InfantrySerialPort: target yaw: {:.3f}, target pitch: {:.3f},nowLatency: {}ms avgLatency: {}ms",
                                yawAngle, pitchAngle, static_cast<int>(mLatency.back() * 1000),
                                static_cast<int>(GlobalSettings::get().latency * 1000)));
            },
        };
    }
};

HUB_REGISTER_CLASS(InfantrySerialPort);