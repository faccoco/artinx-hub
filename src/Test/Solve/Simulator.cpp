#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "Transform.hpp"
#pragma warning(push, 0)
#include <bullet/btBulletDynamicsCommon.h>
#pragma warning(pop)
#include "HeadInfo.hpp"
#include "SimulatorWorldInfo.hpp"
#include "Utility.hpp"
#include <caf/blocking_actor.hpp>
#include <cstdlib>
#include <fmt/format.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <magic_enum.hpp>
#include <random>

struct SimulatorSettings final {
    double step;

    double v0;
    double v0Std;
    double shootInterval;

    double maxTime;
    uint32_t bulletCount;

    double vibrationLinearRange;
    double vibrationAngleRange;  // in degrees
    double spinningSpeed;        // in circles

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

enum class TargetMotionType : uint32_t {
    Static,
    Spinning,
    Translate2D,
    Translate3D,
    SpinningWithVibrationAndCircle,
    Fans,
    Sentry
};

enum class SourceMotionType : uint32_t { Static, Vibration, Translate2D, Translate3D, Sentry, UAV };

class MotionController {
public:
    virtual void step(btMotionState& motionState, double dt) = 0;
};

class StaticMotionController final : public MotionController {
public:
    void step(btMotionState& motionState, double) override {}
};

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

    static char bulletId, armorId, triangleArmorId;

    void initializeTestCase() {
        {
            // Source
            const auto globalSettings = BlackBoard::instance().get<GlobalSettings>({}).value();

            mBulletShape = std::make_unique<btSphereShape>(static_cast<float>(globalSettings.bulletRadius()));
        }

        {
            // SourceMotion
            mSource.first = std::make_unique<btDefaultMotionState>(
                btTransform{ btQuaternion::getIdentity(), btVector3{ 0.0, static_cast<float>(mConfig.sourceHeight), 0.0 } });
            const auto sourceMotionType = magic_enum::enum_cast<SourceMotionType>(mConfig.sourceMotionType).value();
            switch(sourceMotionType) {
                case SourceMotionType::Static:
                    mSource.second = std::make_unique<StaticMotionController>();
                    break;
                default:
                    throw NotImplemented{};
                    break;
            }
        }

        // Target
        {
            const auto targetType = magic_enum::enum_cast<TargetType>(mConfig.targetType).value();
            switch(targetType) {
                case TargetType::Infantry: {
                    mTargetArmors.push_back(std::make_unique<btBoxShape>(btVector3{
                        widthOfSmallArmor * 0.5, heightOfSmallArmor * 0.5, thinnessOfArmor * 0.5 }));  // NOTICE: half extents
                    auto singleArmor = mTargetArmors.back().get();

                    auto armors = std::make_unique<btCompoundShape>(true, 4);

                    for(int32_t i = 0; i < 4; ++i) {
                        // const btQuaternion quat{ static_cast<float>(i * glm::half_pi<double>()),
                        //                         static_cast<float>(angleOfArmorForInfantry), 0 };
                        const auto yaw = i * glm::half_pi<double>();
                        constexpr auto pitch = angleOfArmorForInfantry;
                        const auto rotateQuat = glm::quat{ glm::quatLookAtRH(
                            glm::dvec3{ std::cos(yaw) * std::cos(pitch), std::sin(pitch),
                                        std::sin(yaw) * std::cos(pitch) },
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
                default:
                    throw NotImplemented{};
                    break;
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
                default:
                    throw NotImplemented{};
                    break;
            }

            const btRigidBody::btRigidBodyConstructionInfo info{ 100.0, motion.get(), mTargetArmors.back().get() };
            body = std::make_unique<btRigidBody>(info);
            body->setFlags(btRigidBodyFlags::BT_DISABLE_WORLD_GRAVITY);
            body->setUserPointer(body->getCollisionShape()->getUserPointer());
            mDynamicWorld->addRigidBody(body.get());
        }
    }

public:
    Simulator(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(Simulator).hash_code() } {
        mCollisionConfig = std::make_unique<btDefaultCollisionConfiguration>();
        mCollisionDispatcher = std::make_unique<btCollisionDispatcher>(mCollisionConfig.get());
        mBroadphaseInterface = std::make_unique<btDbvtBroadphase>();
        mConstraintSolver = std::make_unique<btSequentialImpulseConstraintSolver>();
        mDynamicWorld = std::make_unique<btDiscreteDynamicsWorld>(mCollisionDispatcher.get(), mBroadphaseInterface.get(),
                                                                  mConstraintSolver.get(), mCollisionConfig.get());

        initializeTestCase();

        Timer::instance().addTimer(this->address(), 10ms);
    }
    ~Simulator() {
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
        const auto globalSettings = BlackBoard::instance().get<GlobalSettings>({}).value();
        mDynamicWorld->setGravity(btVector3{ 0, static_cast<float>(globalSettings.gForce), 0 });

        std::normal_distribution<double> vGen{ mConfig.v0, std::fmax(mConfig.v0Std, 1e-3) };
        const auto maxVelocity = mConfig.v0 + 3.0 * mConfig.v0Std;
        const auto minVelocity = mConfig.v0 - 3.0 * mConfig.v0Std;

        const auto speedThreshold =
            globalSettings.bullet42mm ? speedThresholdFor42mmA : speedThresholdFor17mm;  // TODO: handle triangle armor

        std::unordered_set<const btRigidBody*> usedBullet;
        std::unordered_map<const btRigidBody*, btVector3> bulletVelocity;

        while(runFlag) {
            for(auto& [_, p] : mBullets) {
                bulletVelocity[p.get()] = p->getLinearVelocity();
                if(mConfig.printBulletPos) {
                    const auto pos = p->getCenterOfMassPosition();
                    CAF_LOG_INFO(fmt::format("bullet {:.2f} {:.2f} {:.2f}", pos.x(), pos.y(), pos.z()));
                }
            }

            // update drag forces

            // step
            mSource.second->step(*mSource.first, mConfig.step);
            std::get<2>(mTarget)->step(*std::get<0>(mTarget), mConfig.step);
            mDynamicWorld->stepSimulation(static_cast<btScalar>(mConfig.step), 10, 0.001f);
            time += mConfig.step;
            
            CAF_LOG_INFO(fmt::format("Simulator time {:.3f}s bullet count {} hited {}", time, bulletCount, hitCount));

            // update world info
            {
                SimulatorWorldInfo info;

                info.lastUpdate =
                    TimePoint{ static_cast<Duration>(static_cast<Clock::rep>(time * Clock::period::den / Clock::period::num)) };

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

                BlackBoard::instance().updateSync(mKey, std::move(info));
                sendAll(simulator_step_atom_v, mKey);
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
                for(int32_t idx = 0; idx < contactsCount; ++idx) {
                    auto& point = manifold->getContactPoint(idx);
                    velocity = std::max(velocity, static_cast<double>(std::fabs(btDot(point.m_normalWorldOnB, speed))));
                }

                if(velocity > speedThreshold) {
                    ++hitCount;

                    const auto pos = bodyB->getCenterOfMassPosition();

                    CAF_LOG_INFO(fmt::format("Hit at ({:.2f},{:.2f},{:.2f}) vel {:.2f}", pos.x(), pos.y(), pos.z(), velocity));
                    mDynamicWorld->removeRigidBody(const_cast<btRigidBody*>(bodyB));
                }
            }

            // update events
            receive([&](shoot_atom, const bool enableGun) { shoot = enableGun; },
                    [&](update_head_atom, Identifier key) { mHeadKey = key; }, [&](const caf::down_msg& x) { runFlag = false; },
                    [&](const caf::exit_msg& x) { runFlag = false; }, [&](timer_atom) {});
            // shoot
            if(shoot && bulletCount < mConfig.bulletCount && time - lastShoot > mConfig.shootInterval) {
                const auto headData = BlackBoard::instance().get<HeadInfo>(mHeadKey);
                Transform<FrameOfReference::Robot, FrameOfReference::Gun> transA{ glm::identity<glm::dmat4>() };
                if(headData.has_value()) {
                    transA = headData.value().transform;
                }

                auto motion = std::make_unique<btDefaultMotionState>();
                btTransform trans;
                mSource.first->getWorldTransform(trans);
                glm::mat4 mat;
                trans.getOpenGLMatrix(glm::value_ptr(mat));
                const auto transB = decltype(SimulatorWorldInfo::posture){ glm::inverse(mat) };

                const auto transform = transB * transA;
                glm::mat4 transformMat = transform.rawInverse();
                glm::quat rotate{ transformMat };
                glm::vec3 origin = transformMat * glm::vec4{ 0.0, 0.0, 0.0, 1.0 };

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
                mBullets.push_back(std::make_pair(std::move(motion), std::move(body)));

                lastShoot = time;
                ++bulletCount;
            }
            if(time - mConfig.maxTime > -1e-4) {
                runFlag = false;
            }
        }

        CAF_LOG_INFO(fmt::format("Expected {} Result {}", mConfig.expectedCount, hitCount));
        if(hitCount < mConfig.expectedCount) {
            CAF_LOG_ERROR("Test failed");
            terminateSystem(*this, false);
        } else
            terminateSystem(*this, true);
    }
};

char Simulator::armorId;
char Simulator::bulletId;
char Simulator::triangleArmorId;

HUB_REGISTER_CLASS(Simulator);
