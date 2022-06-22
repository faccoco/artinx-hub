#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "SimulatorWorldInfo.hpp"
#include "Transform.hpp"
#include "Utility.hpp"
#include <random>

#include "SuppressWarningBegin.hpp"

#include <bullet/btBulletDynamicsCommon.h>
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
        f.field("step", x.step).invariant([](double v) { return v >= 0.001 && v <= 0.01; }), f.field("v0", x.v0),
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

class MotionController {
public:
    virtual void step(btMotionState& motionState, double dt) = 0;
    virtual ~MotionController() = default;
};

class StaticMotionController final : public MotionController {
public:
    void step(btMotionState&, double) override {}
};

class SpinMotionController final : public MotionController {
    double mSpinningSpeed;

public:
    explicit SpinMotionController(const double spinningSpeed) : mSpinningSpeed{ spinningSpeed } {}
    void step(btMotionState& motionState, const double dt) override {
        btTransform transform;
        motionState.getWorldTransform(transform);
        const auto quat = transform.getRotation();
        const auto rotated =
            glm::rotate(glm::quat{ quat.w(), quat.x(), quat.y(), quat.z() },
                        static_cast<float>(glm::two_pi<double>() * mSpinningSpeed * dt), glm::vec3{ 0.0f, 1.0f, 0.0f });
        transform.setRotation(btQuaternion{ rotated.x, rotated.y, rotated.z, rotated.w });
        motionState.setWorldTransform(transform);
    }
};

class LargeCircleMotionController final : public MotionController {
    double mSpinningSpeed;

public:
    explicit LargeCircleMotionController(const double spinningSpeed) : mSpinningSpeed{ spinningSpeed } {}
    void step(btMotionState& motionState, const double dt) override {
        btTransform transform;
        motionState.getWorldTransform(transform);
        const auto quat = transform.getRotation();
        const auto angle = static_cast<float>(glm::two_pi<double>() * mSpinningSpeed * dt);
        const auto rotated =
            glm::rotate(glm::quat{ quat.w(), quat.x(), quat.y(), quat.z() }, angle, glm::vec3{ 0.0f, 1.0f, 0.0f });
        transform.setRotation(btQuaternion{ rotated.x, rotated.y, rotated.z, rotated.w });
        const auto inverse = glm::rotate(glm::identity<glm::quat>(), -angle, glm::vec3{ 0.0f, 1.0f, 0.0f });
        transform.setOrigin(quatRotate(btQuaternion{ inverse.x, inverse.y, inverse.z, inverse.w }, transform.getOrigin()));
        motionState.setWorldTransform(transform);
    }
};

class UAVMotionController final : public MotionController {
    std::default_random_engine mGenerator;
    std::normal_distribution<double> mDistribution{ 0, 1 };

    void step(btMotionState& motionState, const double dt) override {
        const auto xSpeed = mDistribution(mGenerator);
        const auto ySpeed = mDistribution(mGenerator);
        const auto zSpeed = mDistribution(mGenerator);
        btTransform transform;
        motionState.getWorldTransform(transform);
        transform.setOrigin(btVector3(static_cast<btScalar>(static_cast<double>(transform.getOrigin().getX()) + xSpeed * dt),
                                      static_cast<btScalar>(static_cast<double>(transform.getOrigin().getY()) + ySpeed * dt),
                                      static_cast<btScalar>(static_cast<double>(transform.getOrigin().getZ()) + zSpeed * dt)));
        motionState.setWorldTransform(transform);
    }
};

class SentryMotionController final : public MotionController {
    bool mMovingDirection = false;
    void step(btMotionState& motionState, const double dt) override {
        btTransform transform;
        motionState.getWorldTransform(transform);
        const auto x = transform.getOrigin().getX();
        constexpr auto speed = 0.5;
        if(x > 2.0f)
            mMovingDirection = false;
        else if(x < -2.0f)
            mMovingDirection = true;

        transform.setOrigin(btVector3(x + static_cast<float>((mMovingDirection ? speed : -speed) * dt),
                                      transform.getOrigin().getY(), transform.getOrigin().getZ()));
        motionState.setWorldTransform(transform);
    }
};

// FIXME
class Translate2DMotionController final : public MotionController {
    std::default_random_engine mGenerator;
    std::normal_distribution<double> mDistribution{ 0, 1 };

    void step(btMotionState& motionState, const double dt) override {
        const double ySpeed = mDistribution(mGenerator);
        const double zSpeed = mDistribution(mGenerator);
        btTransform transform;
        motionState.getWorldTransform(transform);
        transform.setOrigin(btVector3(transform.getOrigin().getX(),
                                      static_cast<btScalar>(static_cast<double>(transform.getOrigin().getY()) + ySpeed * dt),
                                      static_cast<btScalar>(static_cast<double>(transform.getOrigin().getZ()) + zSpeed * dt)));
        motionState.setWorldTransform(transform);
    }
};

// FIXME
class VibrationMotionController final : public MotionController {
    double mVibrationRange;
    double mT = 0;

public:
    explicit VibrationMotionController(const double range) : mVibrationRange{ range } {}
    void step(btMotionState& motionState, const double dt) override {
        btTransform transform;
        motionState.getWorldTransform(transform);
        mT += dt;
        transform.setOrigin(
            btVector3(static_cast<btScalar>(static_cast<double>(transform.getOrigin().getX() + mVibrationRange * glm::sin(mT))),
                      transform.getOrigin().getY(), transform.getOrigin().getZ()));
        motionState.setWorldTransform(transform);
    }
};

static char bulletId, armorId, triangleArmorId;

class Simulator final : public HubHelper<caf::blocking_actor, SimulatorSettings, simulator_step_atom> {
    Identifier mKey, mHeadKey{};

    std::unique_ptr<btDefaultCollisionConfiguration> mCollisionConfig;
    std::unique_ptr<btCollisionDispatcher> mCollisionDispatcher;
    std::unique_ptr<btBroadphaseInterface> mBroadphaseInterface;
    std::unique_ptr<btSequentialImpulseConstraintSolver> mConstraintSolver;
    std::unique_ptr<btDynamicsWorld> mDynamicWorld;
    std::unique_ptr<btSphereShape> mBulletShape;
    std::vector<std::pair<std::unique_ptr<btMotionState>, std::unique_ptr<btRigidBody>>> mBullets;
    std::vector<std::unique_ptr<btCollisionShape>> mTargetArmors;
    std::tuple<std::unique_ptr<btMotionState>, std::unique_ptr<btRigidBody>, std::unique_ptr<MotionController>> mTarget;
    std::pair<std::unique_ptr<btMotionState>, std::unique_ptr<MotionController>> mSource;
    std::mt19937_64 mEngine{ static_cast<uint64_t>(Clock::now().time_since_epoch().count()) };

    void initializeTestCase() {
        {
            // Source
            const auto& globalSettings = GlobalSettings::get();

            mBulletShape = std::make_unique<btSphereShape>(static_cast<float>(globalSettings.bulletRadius()));
        }

        {
            // SourceMotion
            mSource.first = std::make_unique<btDefaultMotionState>(
                btTransform{ btQuaternion::getIdentity(), btVector3{ 0.0, static_cast<float>(mConfig.sourceHeight), 0.0 } });
            switch(magic_enum::enum_cast<SourceMotionType>(mConfig.sourceMotionType).value()) {
                case SourceMotionType::Static:
                    mSource.second = std::make_unique<StaticMotionController>();
                    break;
                case SourceMotionType::UAV:
                    mSource.second = std::make_unique<UAVMotionController>();
                    break;
                case SourceMotionType::Sentry:
                    mSource.second = std::make_unique<SentryMotionController>();
                    break;
                case SourceMotionType::Translate2D:
                    mSource.second = std::make_unique<Translate2DMotionController>();
                    break;
                case SourceMotionType::Vibration:
                    mSource.second = std::make_unique<VibrationMotionController>(mConfig.vibrationLinearRange);
                    break;
                case SourceMotionType::Translate3D:
                    throw NotImplemented{};
            }
        }

        // Target
        {
            switch(magic_enum::enum_cast<TargetType>(mConfig.targetType).value()) {
                case TargetType::Infantry: {
                    mTargetArmors.push_back(std::make_unique<btBoxShape>(
                        btVector3{ static_cast<float>(widthOfSmallArmor * 0.5), static_cast<float>(heightOfSmallArmor * 0.5),
                                   static_cast<float>(thinnessOfArmor * 0.5) }));  // NOTICE: half extents
                    auto singleArmor = mTargetArmors.back().get();

                    auto armors = std::make_unique<btCompoundShape>(true, 4);

                    for(int32_t i = 0; i < 4; ++i) {
                        // const btQuaternion quat{ static_cast<float>(i * glm::half_pi<double>()),
                        //                         static_cast<float>(angleOfArmorForInfantry), 0 };
                        const auto yaw = i * glm::half_pi<double>();
                        constexpr auto pitch = angleOfArmorForInfantry;
                        const auto rotateQuat = glm::quat{ glm::quatLookAtRH(
                            glm::dvec3{ std::cos(yaw) * std::cos(pitch), std::sin(pitch), std::sin(yaw) * std::cos(pitch) },
                            glm::dvec3{ 0.0, 0.0, 1.0 }) };

                        const btVector3 base{ static_cast<float>(radiusOfInfantry * std::cos(yaw)), 0.0,
                                              static_cast<float>(radiusOfInfantry * std::sin(yaw)) };

                        armors->addChildShape(
                            btTransform{ btQuaternion{ rotateQuat.x, rotateQuat.y, rotateQuat.z, rotateQuat.w }, base },
                            singleArmor);
                    }

                    armors->setUserPointer(&armorId);
                    mTargetArmors.push_back(std::move(armors));
                } break;
                case TargetType::Sentry: {
                    mTargetArmors.push_back(std::make_unique<btBoxShape>(
                        btVector3{ static_cast<float>(widthOfSmallArmor * 0.5), static_cast<float>(heightOfSmallArmor * 0.5),
                                   static_cast<float>(thinnessOfArmor * 0.5) }));  // NOTICE: half extents
                    auto singleArmor = mTargetArmors.back().get();

                    auto armors = std::make_unique<btCompoundShape>(true, 4);

                    for(int32_t i = 0; i < 2; ++i) {
                        // const btQuaternion quat{ static_cast<float>(i * glm::half_pi<double>()),
                        //                         static_cast<float>(angleOfArmorForInfantry), 0 };
                        const auto yaw = i * glm::pi<double>();
                        constexpr auto pitch = angleOfArmorForSentry;
                        const auto rotateQuat = glm::quat{ glm::quatLookAtRH(
                            glm::dvec3{ std::cos(yaw) * std::cos(pitch), std::sin(pitch), std::sin(yaw) * std::cos(pitch) },
                            glm::dvec3{ 0.0, 0.0, 1.0 }) };

                        const btVector3 base{ static_cast<float>(radiusOfInfantry * std::cos(yaw)), 0.0,
                                              static_cast<float>(radiusOfInfantry * std::sin(yaw)) };

                        armors->addChildShape(
                            btTransform{ btQuaternion{ rotateQuat.x, rotateQuat.y, rotateQuat.z, rotateQuat.w }, base },
                            singleArmor);
                    }

                    armors->setUserPointer(&armorId);
                    mTargetArmors.push_back(std::move(armors));
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
            const auto targetMotionType = magic_enum::enum_cast<TargetMotionType>(mConfig.targetMotionType).value();

            auto& [motion, body, controller] = mTarget;

            motion = std::make_unique<btDefaultMotionState>(btTransform{
                btQuaternion::getIdentity(),
                btVector3{ 0, static_cast<float>(mConfig.targetHeight), static_cast<float>(-mConfig.standardDistance) } });

            switch(targetMotionType) {
                case TargetMotionType::Static: {
                    controller = std::make_unique<StaticMotionController>();
                } break;
                case TargetMotionType::Spinning: {
                    controller = std::make_unique<SpinMotionController>(mConfig.spinningSpeed);
                } break;
                case TargetMotionType::LargeCircle: {
                    controller = std::make_unique<LargeCircleMotionController>(mConfig.spinningSpeed);
                } break;
                case TargetMotionType::Translate2D:
                    [[fallthrough]];
                case TargetMotionType::Translate3D:
                    [[fallthrough]];
                case TargetMotionType::Fans:
                    [[fallthrough]];
                case TargetMotionType::Sentry:
                    throw NotImplemented{};
            }

            const btRigidBody::btRigidBodyConstructionInfo info{ 100.0, motion.get(), mTargetArmors.back().get() };
            body = std::make_unique<btRigidBody>(info);
            body->setFlags(btRigidBodyFlags::BT_DISABLE_WORLD_GRAVITY);
            body->setUserPointer(body->getCollisionShape()->getUserPointer());
            mDynamicWorld->addRigidBody(body.get());
        }
    }

public:
    Simulator(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        mCollisionConfig = std::make_unique<btDefaultCollisionConfiguration>();
        mCollisionDispatcher = std::make_unique<btCollisionDispatcher>(mCollisionConfig.get());
        mBroadphaseInterface = std::make_unique<btDbvtBroadphase>();
        mConstraintSolver = std::make_unique<btSequentialImpulseConstraintSolver>();
        mDynamicWorld = std::make_unique<btDiscreteDynamicsWorld>(mCollisionDispatcher.get(), mBroadphaseInterface.get(),
                                                                  mConstraintSolver.get(), mCollisionConfig.get());

        initializeTestCase();

        Timer::instance().addTimer(this->address(), 10ms);
    }
    ~Simulator() override {
        for(int32_t idx = mDynamicWorld->getNumCollisionObjects() - 1; idx >= 0; --idx)
            mDynamicWorld->removeCollisionObject(mDynamicWorld->getCollisionObjectArray()[idx]);
    }

    void act() override {
        double time = 0.0;
        bool runFlag = true;
        bool shoot = false;
        double lastShoot = -2 * mConfig.shootInterval;
        uint32_t hitCount = 0;
        uint32_t bulletCount = 0;
        auto& globalSettings = GlobalSettings::get();
        mDynamicWorld->setGravity(btVector3{ 0, static_cast<float>(globalSettings.gForce), 0 });

        globalSettings.bulletSpeed = mConfig.v0;
        std::normal_distribution vGen{ mConfig.v0, std::fmax(mConfig.v0Std, 1e-3) };
        const auto maxVelocity = mConfig.v0 + 3.0 * mConfig.v0Std;
        const auto minVelocity = mConfig.v0 - 3.0 * mConfig.v0Std;

        // const auto speedThreshold =
        //    globalSettings.bullet42mm ? speedThresholdFor42mmA : speedThresholdFor17mm;  // TODO: handle triangle armor

        constexpr auto speedThreshold = -1.0;  // disable speed threshold

        std::unordered_set<const btRigidBody*> usedBullet;
        std::unordered_map<const btRigidBody*, btVector3> bulletVelocity;

        while(runFlag) {
            for(auto& [_, p] : mBullets) {
                bulletVelocity[p.get()] = p->getLinearVelocity();
                if(mConfig.printBulletPos) {
                    const auto pos = p->getCenterOfMassPosition();
                    logInfo(fmt::format("bullet {:.2f} {:.2f} {:.2f}", pos.x(), pos.y(), pos.z()));
                }
            }

            // update drag forces

            // step
            mSource.second->step(*mSource.first, mConfig.step);
            std::get<2>(mTarget)->step(*std::get<0>(mTarget), mConfig.step);
            mDynamicWorld->stepSimulation(static_cast<btScalar>(mConfig.step), 10, 0.001f);
            time += mConfig.step;

            // update world info
            {
                SimulatorWorldInfo info;

                info.lastUpdate =
                    TimePoint{ static_cast<Duration>(static_cast<Clock::rep>(time * Clock::period::den / Clock::period::num)) };
                SynchronizedClock::instance().setSimulationTime(info.lastUpdate);

                {
                    btTransform trans;
                    mSource.first->getWorldTransform(trans);
                    glm::mat4 mat;
                    trans.getOpenGLMatrix(glm::value_ptr(mat));
                    info.posture = decltype(info.posture){ glm::inverse(mat) };
                }

                {
                    const auto& motion = std::get<0>(mTarget);
                    const auto ptr = mTargetArmors.back().get();
                    const auto shape = reinterpret_cast<btCompoundShape*>(ptr);  // NOTICE: RTTI is not available

                    btTransform trans;
                    motion->getWorldTransform(trans);

                    const auto count = shape->getNumChildShapes();

                    for(int32_t idx = 0; idx < count; ++idx) {
                        const auto& transform = shape->getChildTransform(idx);
                        const auto worldTransform = trans * transform;
                        const auto origin = worldTransform.getOrigin();
                        const auto originWorld = Point<UnitType::Distance, FrameOfReference::Ground>{ glm::dvec3{
                            origin.x(), origin.y(), origin.z() } };

                        info.targets.push_back(originWorld);
                    }
                }

                sendAll(simulator_step_atom_v, BlackBoard::instance().updateSync(mKey, std::move(info)));
            }

            // update collisions
            const auto manifoldsCount = mCollisionDispatcher->getNumManifolds();
            for(int32_t idx = 0; idx < manifoldsCount; ++idx) {
                const auto manifold = mCollisionDispatcher->getManifoldByIndexInternal(idx);
                auto typeA = manifold->getBody0()->getUserPointer();
                auto typeB = manifold->getBody1()->getUserPointer();
                if(typeA == &bulletId && typeB == &bulletId)
                    continue;
                // NOTICE: RTTI is not available
                auto bodyA = reinterpret_cast<const btRigidBody*>(manifold->getBody0());  // armor
                auto bodyB = reinterpret_cast<const btRigidBody*>(manifold->getBody1());  // bullet

                if(typeA == &bulletId) {
                    std::swap(bodyA, bodyB);
                }

                if(usedBullet.count(bodyB)) {
                    continue;
                }

                const auto speed = bulletVelocity[bodyB];
                double velocity = 0.0;

                const auto contactsCount = manifold->getNumContacts();
                for(int32_t i = 0; i < contactsCount; ++i) {
                    auto& point = manifold->getContactPoint(i);
                    velocity = std::max(velocity, static_cast<double>(std::fabs(btDot(point.m_normalWorldOnB, speed))));
                }

                if(velocity > speedThreshold) {
                    ++hitCount;

                    const auto pos = bodyB->getCenterOfMassPosition();

                    logInfo(fmt::format("Hit at ({:.2f},{:.2f},{:.2f}) vel {:.2f}", pos.x(), pos.y(), pos.z(), velocity));
                    mDynamicWorld->removeRigidBody(const_cast<btRigidBody*>(bodyB));
                } else {
                    logInfo(fmt::format("Bad hit: vertical speed = {:.3f}", velocity));
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

            btTransform trans;
            mSource.first->getWorldTransform(trans);
            glm::mat4 mat;
            trans.getOpenGLMatrix(glm::value_ptr(mat));
            const auto transB = decltype(SimulatorWorldInfo::posture){ glm::inverse(mat) };

            const auto transform = transB * transA;

            // shoot
            if(shoot && bulletCount < mConfig.bulletCount && time - lastShoot > mConfig.shootInterval) {
                glm::mat4 transformMat = transform.rawInverse();
                glm::quat rotate{ transformMat };
                glm::vec3 origin = transformMat * glm::vec4{ 0.0, 0.0, 0.0, 1.0 };

                auto motion = std::make_unique<btDefaultMotionState>();
                motion->setWorldTransform(btTransform{ btQuaternion{ rotate.x, rotate.y, rotate.z, rotate.w },
                                                       btVector3{ origin.x, origin.y, origin.z } });

                const btRigidBody::btRigidBodyConstructionInfo info{ static_cast<float>(globalSettings.bulletMass()),
                                                                     motion.get(), mBulletShape.get() };
                auto body = std::make_unique<btRigidBody>(info);
                const auto v = std::clamp(vGen(mEngine), minVelocity, maxVelocity);
                const auto impulse = glm::vec3{ transform(Vector<UnitType::Distance, FrameOfReference::Gun>{
                                                              { 0.0, 0.0, -globalSettings.bulletMass() * v } })
                                                    .raw() };

                body->applyCentralImpulse({ impulse.x, impulse.y, impulse.z });
                mDynamicWorld->addRigidBody(body.get());
                body->setUserPointer(&bulletId);
                mBullets.emplace_back(std::move(motion), std::move(body));

                lastShoot = time;
                ++bulletCount;
            }

            logInfo(fmt::format("Simulator time {:.3f}s bullet count {} hit {} shoot {}", time, bulletCount, hitCount, shoot));
            {
                btTransform src;
                mSource.first->getWorldTransform(src);
                const auto posSrc = src.getOrigin();

                logInfo(fmt::format("Source {:.3f} {:.3f} {:.3f}", posSrc.x(), posSrc.y(), posSrc.z()));

                btTransform dst;
                std::get<0>(mTarget)->getWorldTransform(dst);
                const auto posDst = dst.getOrigin();

                logInfo(fmt::format("Target {:.3f} {:.3f} {:.3f}", posDst.x(), posDst.y(), posDst.z()));

                auto diff = posDst - posSrc;
                diff.normalize();

                logInfo(fmt::format("Ref dir {:.3f} {:.3f} {:.3f}", diff.x(), diff.y(), diff.z()));

                const auto real = transform(Vector<UnitType::Distance, FrameOfReference::Gun>{ { 0.0, 0.0, -1.0f } }).raw();
                logInfo(fmt::format("Gun dir {:.3f} {:.3f} {:.3f}", real.x, real.y, real.z));

                const auto& motion = std::get<0>(mTarget);
                const auto ptr = mTargetArmors.back().get();
                const auto shape = reinterpret_cast<btCompoundShape*>(ptr);  // NOTICE: RTTI is not available

                motion->getWorldTransform(trans);

                btScalar minDist = 1e10f;
                std::optional<std::pair<btVector3, btVector3>> closest = std::nullopt;

                const auto count = shape->getNumChildShapes();
                for(int32_t idx = 0; idx < count; ++idx) {
                    const auto& transformShape = shape->getChildTransform(idx);
                    const auto worldTransform = trans * transformShape;
                    const auto origin = worldTransform.getOrigin();

                    for(auto& [bulletTrans, _] : mBullets) {
                        btTransform trans2;
                        bulletTrans->getWorldTransform(trans2);

                        const auto origin2 = trans2.getOrigin();
                        if(const auto dist = btDistance(origin, origin2); dist < minDist) {
                            minDist = dist;
                            closest = { origin, origin2 };
                        }
                    }
                }

                if(closest) {
                    const auto [p1, p2] = closest.value();
                    logInfo(fmt::format("Closest pair armor {:.3f} {:.3f} {:.3f} <-> bullet {:.3f} {:.3f} {:.3f} : {:.3f} m",
                                        p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), minDist));
                }
            }

            if(time - mConfig.maxTime > -1e-4) {
                runFlag = false;
            }
            if(hitCount == mConfig.bulletCount) {
                runFlag = false;
            }
            std::this_thread::sleep_for(5ms);
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
