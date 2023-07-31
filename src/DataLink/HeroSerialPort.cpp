#include "BlackBoard.hpp"
#include "HeadInfo.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"
#include "SerialPort/PacketHelper.hpp"
#include "SerialPort/SerialPort.hpp"

#include <caf/event_based_actor.hpp>
#include <glm/ext/matrix_transform.hpp>

class HeroRecvPacket final {
public:
    static constexpr uint16_t id = 0x0A;

    float yaw, pitch, roll, downYaw, downPitch, bulletSpeed, speedX, speedY;
    bool color, shooterId, energyMode, periodMode, priorMode;
    float capEnergy, chasisPower;
    uint16_t shootDelayTime;  // ms
    explicit HeroRecvPacket(std::array<uint8_t, 1024>& buffer) {
        PacketReader<1024> reader(buffer);
        yaw = reader.readCompressedFloat(-4.0f, 0.0005f);
        pitch = reader.readCompressedFloat(-4.0f, 0.0005f);
        roll = reader.readCompressedFloat(-4.0f, 0.0005f);
        downYaw = reader.readCompressedFloat(-4.0f, 0.0005f);
        downPitch = reader.readCompressedFloat(-4.0f, 0.0005f);
        speedX = reader.readCompressedFloat(-20.0f, 0.01f);
        speedY = reader.readCompressedFloat(-20.0f, 0.01f);
        const auto mask = reader.read();
        color = mask & 1;
        periodMode = (mask >> 4) & 1;
        priorMode = (mask >> 5) & 1;
        bulletSpeed = reader.readCompressedFloat(-1.0f, 0.005f);
        capEnergy = reader.readCompressedFloat(-1.0f, 0.1f);
        chasisPower = reader.readCompressedFloat(-1.0f, 0.01f);
        shootDelayTime = reader.read<uint16_t>();
    }
};

class HeroSendPacket final {
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

struct HeroSerialPortSettings final {
    std::string devPath;
    uint32_t baudRate;
    double minBulletSpeed;
    double maxBulletSpeed;
    bool getBulletSpeedFromSerial;
};

template <class Inspector>
bool inspect(Inspector& f, HeroSerialPortSettings& x) {
    return f.object(x).fields(f.field("devPath", x.devPath), f.field("baudRate", x.baudRate),
                              f.field("minBulletSpeed", x.minBulletSpeed).fallback(7.0),
                              f.field("maxBulletSpeed", x.maxBulletSpeed).fallback(16.0),
                              f.field("getBulletSpeedFromSerial", x.getBulletSpeedFromSerial).fallback(false));
}

class HeroSerialPort final : public HubHelper<caf::event_based_actor, HeroSerialPortSettings, update_head_atom,
                                              update_posture_atom, hero_strategy_control_atom>,
                             public SerialPort<HeroRecvPacket, HeroSendPacket> {
    Identifier mKey;

    constexpr static size_t latencyLen = 100;
    constexpr static size_t mShootDelayLen = 5;
    constexpr static std::uint16_t maxShootDelay = 500;  // ms
    constexpr static size_t mBulletSpeedLen = 5;

    std::atomic<float> mCapEnergy, mChasisPower;

    bool mPeriodMode = false;

    std::deque<double> mLatency;
    std::deque<uint16_t> mShootDelay;
    std::deque<double> mBulletSpeed;
    std::optional<double> mLastBulletSpeed;

    TimePoint mLastTargetTime;

    void heroRecvCB(const HeroRecvPacket& fdb) {
        // bullet speed
        // new value
        if(!mLastBulletSpeed.has_value() || mLastBulletSpeed.value() != fdb.bulletSpeed) {
            HubLogger::electricCtrlLog(fmt::format("bulletSpeed: {}", fdb.bulletSpeed));
            // valid value
            if(mConfig.getBulletSpeedFromSerial && fdb.bulletSpeed > mConfig.minBulletSpeed &&
               fdb.bulletSpeed < mConfig.maxBulletSpeed) {
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

        // shoot delay
        //     if((!mShootDelay.empty()) && (fdb.shootDelayTime != mShootDelay.back()))
        //          logInfo(fmt::format("shoot delay {}", fdb.shootDelayTime));
        // new value
        if(mShootDelay.empty() || fdb.shootDelayTime != mShootDelay.back()) {
            // valid value
            if(fdb.shootDelayTime < maxShootDelay) {
                if(mShootDelay.size() >= mShootDelayLen)
                    mShootDelay.pop_front();
                mShootDelay.push_back(fdb.shootDelayTime);
                GlobalSettings::get().shootDelayTime = avg(mShootDelay) / 1000.0;  // ms -> s
            }
            HubLogger::visualLog(fmt::format("shoot delay: fdb:{:.3f}s avg:{:.3f}s", fdb.shootDelayTime / 1000.0,
                                             GlobalSettings::get().shootDelayTime));
        }

        HubLogger::watch("bullet speed", GlobalSettings::get().bulletSpeed);
        // HubLogger::watch("fdb bullet speed", fdb.bulletSpeed);
        // HubLogger::watch("fdb shoot delay time", fdb.shootDelayTime);
        // HubLogger::watch("shoot delay time", static_cast<int>(GlobalSettings::get().shootDelayTime * 1000));

        auto deltaYaw1 = mSendPacket.yaw - fdb.yaw;
        auto deltaPitch1 = mSendPacket.pitch - fdb.pitch;
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

        // enter period mode
        if(!mPeriodMode && fdb.periodMode) {
            std::lock_guard lock{ mPacketMutex };
            mPeriodMode = true;
            mSendPacket.yaw = fdb.yaw;
            mSendPacket.pitch = fdb.pitch;
            mSendPacket.isFire = false;
        }
        mPeriodMode = fdb.periodMode;
        sendAll(hero_strategy_control_atom_v, fdb.periodMode, fdb.priorMode);
        HubLogger::watch("period mode", fdb.periodMode);
        HubLogger::watch("prior mode", fdb.priorMode);

        mCapEnergy = fdb.capEnergy;
        mChasisPower = fdb.chasisPower;

        const HeadInfo infoUp{ SynchronizedClock::instance().now(), { -fdb.roll, fdb.pitch, fdb.yaw } };

        PostureData posture;
        posture.lastUpdate = SynchronizedClock::instance().now();
        posture.tfGround2Robot = Transform<FrameOfRef::Ground, FrameOfRef::Robot>{ glm::identity<glm::dmat4>() };
        posture.linearVelocityOfRobot = Vector<UnitType::LinearVelocity, FrameOfRef::Ground>{ { fdb.speedX, 0, -fdb.speedY } };

        sendAll(update_posture_atom_v, BlackBoard::instance().updateSync(mKey, posture));
        sendMasked(update_head_atom_v, 1U, 1U, BlackBoard::instance().updateSync(mKey, infoUp));
    }

    void heroSetPacket() {
        mSendPacket.hasTargets = 0;
        if(mPeriodMode || Clock::now() - mLastTargetTime < 500ms)
            mSendPacket.hasTargets |= 1;
        HubLogger::watch("hasTargets", mSendPacket.hasTargets);
    }

public:
    HeroSerialPort(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, std::move(name) }, SerialPort<HeroRecvPacket, HeroSendPacket>(
                                                          mConfig.devPath, mConfig.baudRate,
                                                          std::bind(&HeroSerialPort::heroRecvCB, this, std::placeholders::_1),
                                                          std::bind(&HeroSerialPort::heroSetPacket, this)),
          mKey(generateKey(this)) {}

    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](set_target_info_atom, GroupMask mask, Clock::rep begin, double yawAngle, double pitchAngle, bool isFire,
                   SolverType solverType) {
                ACTOR_PROTOCOL_CHECK(set_target_info_atom, GroupMask, Clock::rep, double, double, bool, SolverType);

                yawAngle = normalizeAngle(yawAngle - glm::half_pi<double>());

                isFire = solverType && isFire;
                {
                    std::lock_guard lock{ mPacketMutex };
                    if(mPeriodMode && solverType == normalSolver)
                        return;

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
                    fmt::format("HeroSerialPort: target yaw: {:.3f}, target pitch: {:.3f},nowLatency: {}ms avgLatency: {}ms",
                                yawAngle, pitchAngle, static_cast<int>(mLatency.back() * 1000),
                                static_cast<int>(GlobalSettings::get().latency * 1000)));
            },
        };
    }
};

HUB_REGISTER_CLASS(HeroSerialPort);