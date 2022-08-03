#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "SimulatorWorldInfo.hpp"
#include "Transform.hpp"
#include "Utility.hpp"
#include <random>

#include "SuppressWarningBegin.hpp"

#include <caf/blocking_actor.hpp>
#include <fmt/format.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"

struct SimulatorSettings final {
    double step;

    double v0;
    double v0Std;
    double shootInterval;

    double maxTime;
    uint32_t bulletCount;

    double vibrationLinearRange;
    double vibrationAngleRange;  // in degrees
    double spinningSpeed;        // in circles/s

    double standardDistance;
    double sourceHeight;
    double targetHeight;

    std::string targetType;
    std::string targetMotionType;
    std::string sourceMotionType;

    uint32_t expectedCount;

    bool printBulletPos;
};

template <class Inspector>
bool inspect(Inspector& f, SimulatorSettings& x) {
    return f.object(x).fields(
        f.field("step", x.step).invariant([](const double v) { return v >= 0.001 && v <= 0.01; }), f.field("v0", x.v0),
        f.field("v0Std", x.v0Std), f.field("shootInterval", x.shootInterval), f.field("maxTime", x.maxTime),
        f.field("bulletCount", x.bulletCount), f.field("vibrationLinearRange", x.vibrationLinearRange),
        f.field("vibrationAngleRange", x.vibrationAngleRange), f.field("spinningSpeed", x.spinningSpeed),
        f.field("standardDistance", x.standardDistance), f.field("sourceHeight", x.sourceHeight),
        f.field("targetHeight", x.targetHeight), f.field("targetType", x.targetType),
        f.field("targetMotionType", x.targetMotionType), f.field("sourceMotionType", x.sourceMotionType),
        f.field("expectedCount", x.expectedCount), f.field("printBulletPos", x.printBulletPos).fallback(false));
}

enum class TargetType : uint32_t { Infantry, Hero, BalancedInfantry, Sentry, Outpost, BaseClosed, BaseExpanded, Fans };

enum class TargetMotionType : uint32_t { Static, Spinning, Translate2D, Translate3D, LargeCircle, Fans, Sentry };

enum class SourceMotionType : uint32_t { Static, Vibration, Translate2D, Translate3D, Sentry, UAV };

using MotionState = glm::dmat4;

class MotionController {
public:
    virtual void step(MotionState& motionState, double dt) = 0;
    virtual ~MotionController() = default;
};

class StaticMotionController final : public MotionController {
public:
    void step(MotionState&, double) override {}
};

class SpinMotionController final : public MotionController {
    double mSpinningSpeed;

public:
    explicit SpinMotionController(const double spinningSpeed) : mSpinningSpeed{ spinningSpeed } {}
    void step(MotionState& motionState, const double dt) override {
        motionState =
            MotionState{ glm::rotate(motionState, glm::two_pi<double>() * mSpinningSpeed * dt, glm::dvec3{ 0.0, 1.0, 0.0 }) };
    }
};



class LargeCircleMotionController final : public MotionController {
    double mSpinningSpeed;

public:
    explicit LargeCircleMotionController(const double spinningSpeed) : mSpinningSpeed{ spinningSpeed } {}
    void step(MotionState& motionState, const double dt) override {
        motionState =
            MotionState{ glm::rotate(motionState, glm::two_pi<double>() * mSpinningSpeed * dt, glm::dvec3{ 0.0, 1.0, 0.0 }) };
    }
};

class SentryMotionController final : public MotionController {
    bool mMovingDirection = false;
    void step(MotionState& motionState, const double dt) override {
        const auto translation = motionState * glm::dvec4{ 0.0, 0.0, 0.0, 1.0 };
        const auto x = translation.x;
        constexpr auto speed = 0.5;
        if(x > 2.0)
            mMovingDirection = false;
        else if(x < -2.0)
            mMovingDirection = true;

        motionState =
            MotionState{ glm::translate(motionState, glm::dvec3{ (mMovingDirection ? speed : -speed) * dt, 0.0, 0.0 }) };
    }
};

class Translate2DMotionController final : public MotionController {
    static constexpr double alpha = 0.001;

    std::default_random_engine mGenerator;
    double mMaxV;
    std::normal_distribution<double> mDistribution;
    double mSpeedX, mSpeedZ, mT = 0.0;
    double mSmoothX = 0.0, mSmoothZ = 0.0;

    void updateSpeed(const double dt) {
        mT += dt;
        if(mT >= 2.0) {
            mSpeedX = std::clamp(mDistribution(mGenerator), -mMaxV, mMaxV);
            mSpeedZ = std::clamp(mDistribution(mGenerator), -mMaxV, mMaxV);

            mT -= 2.0;
        }

        mSmoothX = (1.0 - alpha) * mSmoothX + alpha * mSpeedX;
        mSmoothZ = (1.0 - alpha) * mSmoothZ + alpha * mSpeedZ;
    }

public:
    explicit Translate2DMotionController(const double maxV) : mMaxV{ maxV }, mDistribution{ 0, maxV / 3.0 } {
        updateSpeed(2.0 + 1e-5);
    }

    void step(MotionState& motionState, const double dt) override {
        updateSpeed(dt);

        motionState = MotionState{ glm::translate(motionState, glm::dvec3{ mSmoothX * dt, 0.0, mSmoothZ * dt }) };
    }
};

class Simulator final : public HubHelper<caf::blocking_actor, SimulatorSettings, simulator_step_atom> {
    Identifier mKey, mHeadKey{};

    std::vector<std::pair<glm::dvec3, glm::dvec3>> mBullets;    //[pose velocity]
    std::pair<MotionState, std::unique_ptr<MotionController>> mTarget;
    std::vector<std::pair<MotionState, double>> mTargetArmors;
    std::pair<MotionState, std::unique_ptr<MotionController>> mSource;
    std::mt19937_64 mEngine{ static_cast<uint64_t>(Clock::now().time_since_epoch().count()) };

    void initializeTestCase() {
        {
            // SourceMotion
            mSource.first = MotionState{ glm::translate(glm::identity<glm::dmat4>(), { 0.0, mConfig.sourceHeight, 0.0 }) };

            switch(magic_enum::enum_cast<SourceMotionType>(mConfig.sourceMotionType).value()) {
                case SourceMotionType::Static:
                    mSource.second = std::make_unique<StaticMotionController>();
                    break;

                case SourceMotionType::Sentry:
                    mSource.second = std::make_unique<SentryMotionController>();
                    break;
                case SourceMotionType::Translate2D:
                    mSource.second = std::make_unique<Translate2DMotionController>(mConfig.vibrationLinearRange);
                    break;
                case SourceMotionType::UAV:
                    [[fallthrough]];
                case SourceMotionType::Vibration:
                    [[fallthrough]];
                case SourceMotionType::Translate3D:
                    throw NotImplemented{};
            }
        }

        // Target
        {
            switch(magic_enum::enum_cast<TargetType>(mConfig.targetType).value()) {
                case TargetType::Infantry: {
                    for(uint32_t i = 0; i < 4; ++i) {
                        mTargetArmors.emplace_back(
                            glm::translate(glm::rotate(glm::identity<glm::dmat4>(),
                                                       glm::half_pi<double>() * static_cast<double>(i), { 0.0, 1.0, 0.0 }),
                                           { 0.0, 0.0, radiusOfInfantry }),
                            std::sqrt(widthOfSmallArmor * heightOfSmallArmor) * 0.5);
                    }
                } break;
                case TargetType::Sentry: {
                    for(uint32_t i = 0; i < 2; ++i) {
                        mTargetArmors.emplace_back(
                            glm::translate(glm::rotate(glm::identity<glm::dmat4>(), glm::pi<double>() * static_cast<double>(i),
                                                       { 0.0, 1.0, 0.0 }),
                                           { 0.0, 0.0, radiusOfInfantry * 0.5 }),
                            std::sqrt(widthOfLargeArmor * heightOfLargeArmor) * 0.5);
                    }
                } break;

                case TargetType::Hero:
                    [[fallthrough]];
                case TargetType::BalancedInfantry:
                    [[fallthrough]];
                case TargetType::BaseClosed:
                    [[fallthrough]];
                case TargetType::BaseExpanded:
                    [[fallthrough]];
                case TargetType::Fans:
                    [[fallthrough]];
                case TargetType::Outpost:
                    throw NotImplemented{};
            }
        }

        // TargetMotion
        {
            auto& [motion, controller] = mTarget;
            motion = MotionState{ glm::translate(glm::identity<glm::dmat4>(),
                                                 { 0.0, mConfig.targetHeight, -mConfig.standardDistance }) };

            switch(magic_enum::enum_cast<TargetMotionType>(mConfig.targetMotionType).value()) {
                case TargetMotionType::Static: {
                    controller = std::make_unique<StaticMotionController>();
                } break;
                case TargetMotionType::Spinning: {
                    controller = std::make_unique<SpinMotionController>(mConfig.spinningSpeed);
                } break;
                case TargetMotionType::LargeCircle: {
                    controller = std::make_unique<LargeCircleMotionController>(mConfig.spinningSpeed);
                } break;
                case TargetMotionType::Translate2D: {
                    controller = std::make_unique<Translate2DMotionController>(mConfig.vibrationLinearRange);
                } break;
                case TargetMotionType::Translate3D:
                    [[fallthrough]];
                case TargetMotionType::Fans:
                    [[fallthrough]];
                case TargetMotionType::Sentry:
                    throw NotImplemented{};
            }
        }
    }

public:
    Simulator(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        initializeTestCase();

        Timer::instance().addTimer(this->address(), 10ms);
    }

    void act() override {
        double time = 0.0;
        bool runFlag = true;
        bool shoot = false;
        double lastShoot = -2 * mConfig.shootInterval;
        uint32_t hitCount = 0;
        uint32_t bulletCount = 0;
        auto& globalSettings = GlobalSettings::get();
        const auto dt = mConfig.step;

        globalSettings.bulletSpeed = mConfig.v0;
        std::normal_distribution vGen{ mConfig.v0, std::fmax(mConfig.v0Std, 1e-3) };
        const auto maxVelocity = mConfig.v0 + 3.0 * mConfig.v0Std;
        const auto minVelocity = mConfig.v0 - 3.0 * mConfig.v0Std;
        const auto bulletRadius = GlobalSettings::get().bulletRadius();

        while(runFlag) {
            for(auto& [pos, v] : mBullets) {
                if(pos.y < 0.0)
                    continue;

                if(mConfig.printBulletPos) {
                    logInfo(fmt::format("bullet {:.2f} {:.2f} {:.2f}", pos.x, pos.y, pos.z));
                }
            }

            // update drag forces

            // step:update source pose and velocity
            auto vSrc = glm::zero<glm::dvec3>();
            {
                const auto p1 = mSource.first * glm::dvec4{ 0.0, 0.0, 0.0, 1.0 };
                mSource.second->step(mSource.first, dt);
                const auto p2 = mSource.first * glm::dvec4{ 0.0, 0.0, 0.0, 1.0 };
                vSrc = (p2 - p1) / dt;
            }
            mTarget.second->step(mTarget.first, dt);

            for(auto& [pos, v] : mBullets) {
                if(pos.y < 0.0)
                    continue;
                pos += v * dt;
                v += glm::dvec3{ 0.0, GlobalSettings::get().gForce * dt, 0.0 };
            }

            time += dt;

            // update world info
            {
                SimulatorWorldInfo info;

                info.lastUpdate =
                    TimePoint{ static_cast<Duration>(static_cast<Clock::rep>(time * Clock::period::den / Clock::period::num)) };
                SynchronizedClock::instance().setSimulationTime(info.lastUpdate);

                info.posture = decltype(info.posture){ glm::inverse(mSource.first) };

                {
                    const auto& motion = mTarget.first;

                    for(auto& trans : mTargetArmors) {
                        const auto pos = motion * trans.first * glm::dvec4{ 0.0, 0.0, 0.0, 1.0 };
                        info.targets.emplace_back(pos);
                    }
                }

                sendAll(simulator_step_atom_v, BlackBoard::instance().updateSync(mKey, std::move(info)));
            }

            // update collisions
            {
                const auto& motion = mTarget.first;
                for(auto& trans : mTargetArmors) {
                    const auto pos = glm::dvec3{ motion * (trans.first * glm::dvec4{ 0.0, 0.0, 0.0, 1.0 }) };

                    const auto d = trans.second + bulletRadius;
                    for(auto& [p, v] : mBullets) {
                        if(p.y < 0.0)
                            continue;
                        if(glm::distance(p, pos) < d) {
                            p.y = -1.0;
                            logInfo(fmt::format("Hit at ({:.2f},{:.2f},{:.2f})", pos.x, pos.y, pos.z));
                            ++hitCount;
                        }
                    }
                }
            }

            // update events
            receive(
                [&](set_target_info_atom, GroupMask, Clock::rep, const double, const double, const bool isFire) {
                    ACTOR_PROTOCOL_CHECK(set_target_info_atom, GroupMask, Clock::rep, double, double, bool);
                    shoot = isFire;
                },
                [&](update_head_atom, GroupMask, Identifier key) {
                    ACTOR_PROTOCOL_CHECK(update_head_atom, GroupMask, TypedIdentifier<HeadInfo>);
                    mHeadKey = key;
                },
                [&](const caf::down_msg&) { runFlag = false; }, [&](const caf::exit_msg&) { runFlag = false; },
                [&](timer_atom) { ACTOR_PROTOCOL_CHECK(timer_atom); });

            const auto headData = BlackBoard::instance().get<HeadInfo>(mHeadKey);
            Transform<FrameOfReference::Robot, FrameOfReference::Gun> transA{ glm::identity<glm::dmat4>() };
            if(headData.has_value()) {
                transA = headData.value().transform;
            }

            const auto transB = decltype(SimulatorWorldInfo::posture){ glm::inverse(mSource.first) };
            const auto transform = transB * transA;

            // shoot
            if(shoot && bulletCount < mConfig.bulletCount && time - lastShoot > mConfig.shootInterval) {
                const glm::dmat4 transformMat = transform.rawInverse();
                const glm::dvec3 origin = transformMat * glm::dvec4{ 0.0, 0.0, 0.0, 1.0 };

                const auto v = std::clamp(vGen(mEngine), minVelocity, maxVelocity);
                GlobalSettings::get().bulletSpeed = v;

                const auto velocity =
                    glm::normalize(transform(Vector<UnitType::Distance, FrameOfReference::Gun>{ { 0.0, 0.0, -1.0f } }).raw()) * v;

                mBullets.emplace_back(origin, vSrc + velocity);

                lastShoot = time;
                ++bulletCount;
            }

            logInfo(fmt::format("Simulator time {:.3f}s bullet count {} hit {} shoot {}", time, bulletCount, hitCount, shoot));
            {
                const auto posSrc = mSource.first * glm::dvec4{ 0.0, 0.0, 0.0, 1.0 };

                logInfo(fmt::format("Source {:.3f} {:.3f} {:.3f}", posSrc.x, posSrc.y, posSrc.z));

                const auto posDst = mTarget.first * glm::dvec4{ 0.0, 0.0, 0.0, 1.0 };

                logInfo(fmt::format("Target {:.3f} {:.3f} {:.3f}", posDst.x, posDst.y, posDst.z));

                auto diff = glm::normalize(posDst - posSrc);

                logInfo(fmt::format("Ref dir {:.3f} {:.3f} {:.3f}", diff.x, diff.y, diff.z));

                const auto real =
                    glm::normalize(transform(Vector<UnitType::Distance, FrameOfReference::Gun>{ { 0.0, 0.0, -1.0f } }).raw());
                logInfo(fmt::format("Gun dir {:.3f} {:.3f} {:.3f}", real.x, real.y, real.z));

                const auto& motion = mTarget.first;
                double minDist = 1e10;
                std::optional<std::pair<glm::dvec3, glm::dvec3>> closest = std::nullopt;

                for(auto& trans : mTargetArmors) {
                    const auto pos = glm::dvec3{ motion * (trans.first * glm::dvec4{ 0.0, 0.0, 0.0, 1.0 }) };

                    for(auto& [p, v] : mBullets) {
                        if(p.y < 0.0)
                            continue;
                        if(const auto dist = glm::distance(p, pos); dist < minDist) {
                            minDist = dist;
                            closest = { pos, p };
                        }
                    }
                }

                if(closest) {
                    const auto [p1, p2] = closest.value();
                    logInfo(fmt::format("Closest pair armor {:.3f} {:.3f} {:.3f} <-> bullet {:.3f} {:.3f} {:.3f} : {:.3f} m",
                                        p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, minDist));
                }
            }

            if(time - mConfig.maxTime > -1e-4) {
                runFlag = false;
            }
            if(runFlag && bulletCount == mConfig.bulletCount) {
                runFlag = false;
                for(auto& [p, v] : mBullets) {
                    if(p.y >= 0.0) {
                        runFlag = true;
                        break;
                    }
                }
            }
            std::this_thread::sleep_for(10ms);
        }

        logInfo(fmt::format("Expected {} Result {}", mConfig.expectedCount, hitCount));
        appendTestResult(fmt::format("Result {}/{} (Require {}, Shoot {})", hitCount, mConfig.bulletCount, mConfig.expectedCount,
                                     bulletCount));

        if(hitCount < mConfig.expectedCount) {
            logError("Test failed");
            terminateSystem(*this, false);
        } else
            terminateSystem(*this, true);
    }
};

HUB_REGISTER_CLASS(Simulator);
