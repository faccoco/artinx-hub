// Ref: https://github.com/chenjunnn/rm_auto_aim

#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "EKF.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"

#include "SuppressWarningBegin.hpp"
#include "Utility.hpp"

#include <caf/event_based_actor.hpp>
#include <magic_enum.hpp>

struct CarPredictorSettings final {
    bool enablePredictor;
    double maxMatchDist;
    double maxSameArmorDist;
    int trackingThreshold;
    int lostThreshold;
    std::vector<double> Q;  // Process covariance matrix
    std::vector<double> R;  // Measurement covariance mat
};

template <class Inspector>
bool inspect(Inspector& f, CarPredictorSettings& x) {
    return f.object(x).fields(
        f.field("enablePredictor", x.enablePredictor), f.field("maxMatchDist", x.maxMatchDist).fallback(0.1), f.field("maxSameArmorDist", x.maxSameArmorDist).fallback(0.3),
        f.field("trackingThreshold", x.trackingThreshold).fallback(5), f.field("lostThreshold", x.lostThreshold).fallback(5),
        f.field("Q", x.Q).invariant([](auto& c) { return c.size() == 9; }).fallback(std::vector<double>(9, 0)),
        f.field("R", x.R).invariant([](auto& c) { return c.size() == 4; }).fallback(std::vector<double>(4, 0)));
}

class CarPredictor final : public HubHelper<caf::event_based_actor, CarPredictorSettings, car_predict_atom> {
    Identifier mKey, mIMUKey;

    enum class TrackingState {
        LOST,
        DETECTING,
        TRACKING,
        TEMP_LOST,
    };
    ExtendedKalmanFilter mEKF;
    struct TrackedArmor {
        TimePoint lastUpdate;
        Eigen::VectorXd state;
        double yaw;
        int32_t id;
        TrackingState trackingState;
    } mTrackedArmor;
    double mLastY = 0.0, mLastR = 0.2;
    int mDetectCount = 0, mLostCount = 0;
    double mDt = 0.01;

    Transform<FrameOfRef::Gun, FrameOfRef::Robot, true> mTfGun2Robot;

    glm::dvec3 getArmorPos(const DetectedTarget& armor) {
        return mTfGun2Robot(Vector<UnitType::Distance, FrameOfRef::Gun>(armor.center.mVal)).mVal;
    }

    double getArmorYaw(const DetectedTarget& armor) {
        auto rmat = combine(mTfGun2Robot, armor.rmat);
        return normalizeAngle(-atan2(rmat.raw()[2][0], rmat.raw()[2][2]) - glm::half_pi<double>());
    }

    double orientationToYaw(double yaw) {
        // Make yaw change continuous
        mTrackedArmor.yaw = mTrackedArmor.yaw + normalizeAngle(yaw - mTrackedArmor.yaw);
        return mTrackedArmor.yaw;
    }

    glm::dvec3 getArmorPosFromState(const Eigen::VectorXd& x) {
        // Calculate predicted position of the current armor
        double xc = x(0), ya = x(1), zc = x(2);
        double yaw = x(3), r = x(8);
        double xa = xc + r * cos(yaw);
        double za = zc + r * sin(yaw);
        return glm::dvec3{ xa, ya, za };
    }

    void handleArmorJump(const glm::dvec3& targetPos, double targetYaw) {
        double yaw = orientationToYaw(targetYaw);
        if(std::fabs(yaw - mTrackedArmor.state(3)) > 0.3) {
            mLastY = mTrackedArmor.state(1);
            mTrackedArmor.state(1) = targetPos.y;
            mTrackedArmor.state(3) = yaw;
            std::swap(mTrackedArmor.state(8), mLastR);
            logInfo("ArmorPredictor: Armor jump to another armor!");
            HubLogger::VisualLog(fmt::format("EKF Armor jump to another armor"));
        }
        auto dist = glm::distance(targetPos, getArmorPosFromState(mTrackedArmor.state));
        if(dist > mConfig.maxMatchDist) {
            mTrackedArmor.state(0) = targetPos.x - mTrackedArmor.state(8) * cos(yaw);
            mTrackedArmor.state(2) = targetPos.z - mTrackedArmor.state(8) * sin(yaw);
            mTrackedArmor.state(4) = 0;
            mTrackedArmor.state(5) = 0;
            mTrackedArmor.state(6) = 0;
            logInfo("ArmorPredictor: The same Armor match distance too far. State wrong, reset EKF");
            HubLogger::VisualLog(fmt::format("ArmorPredictor: The same Armor match distance {} too far. State wrong, reset EKF", dist));
        }
        mEKF.setState(mTrackedArmor.state);
    }

    void handleTrackingState(bool matched) {
        switch(mTrackedArmor.trackingState) {
            case TrackingState::DETECTING: {
                if(matched) {
                    mDetectCount++;
                    if(mDetectCount > mConfig.trackingThreshold) {
                        mDetectCount = 0;
                        mTrackedArmor.trackingState = TrackingState::TRACKING;
                    }
                } else {
                    mDetectCount = 0;
                    mTrackedArmor.trackingState = TrackingState::LOST;
                }
                break;
            }
            case TrackingState::TRACKING: {
                if(!matched) {
                    mTrackedArmor.trackingState = TrackingState::TEMP_LOST;
                    mLostCount++;
                }
                break;
            }
            case TrackingState::TEMP_LOST: {
                if(!matched) {
                    mLostCount++;
                    if(mLostCount > mConfig.lostThreshold) {
                        mLostCount = 0;
                        mTrackedArmor.trackingState = TrackingState::LOST;
                    }
                } else {
                    mTrackedArmor.trackingState = TrackingState::TRACKING;
                    mLostCount = 0;
                }
                break;
            }
            default:
                break;
        }
        logInfo(fmt::format("ArmorPredictor: tracking state: {}, detectCount: {}, lostCount: {}",
                            magic_enum::enum_name(mTrackedArmor.trackingState), mDetectCount, mLostCount));
        HubLogger::VisualLog(fmt::format("ArmorPredictor: tracking state: {}, detectCount: {}, lostCount: {}",
                                         magic_enum::enum_name(mTrackedArmor.trackingState), mDetectCount, mLostCount));
    }

    void init(const DetectedTarget& armor) {
        mTrackedArmor.yaw = 0;
        double yaw = orientationToYaw(getArmorYaw(armor));
        // Set initial position at 0.2m behind the target
        double r = 0.2;
        auto p = getArmorPos(armor);
        double x = p.x - r * cos(yaw);
        double y = p.y;
        double z = p.z - r * sin(yaw);
        mLastY = y, mLastR = r;
        mTrackedArmor.state << x, y, z, yaw, 0, 0, 0, 0, r;

        mEKF.setState(mTrackedArmor.state);
        logInfo("ArmorPredictor: Init EKF!");
        mTrackedArmor.id = static_cast<int>(armor.id);
        mTrackedArmor.trackingState = TrackingState::DETECTING;
        HubLogger::VisualLog(fmt::format("CarPredictor Init EKF, target: id {}, armor pos ({:.3f}, {:.3f}, {:.3f} yaw {:.3f})",
                                         mTrackedArmor.id, p.x, p.y, p.z, yaw));
    }

    bool update(const double dt, const std::vector<DetectedTarget>& armors) {
        mDt = dt;
        Eigen::VectorXd ekfPrediction = mEKF.predict();
        bool matched = false;
        // Use KF prediction as default target state if no matched armor is found
        mTrackedArmor.state = ekfPrediction;

        if(!armors.empty()) {
            // pair[pos,yaw]
            std::pair<glm::dvec3, double> candidate;
            auto predictedPosition = getArmorPosFromState(ekfPrediction);
            // Difference of the current armor position and tracked armor's predicted position
            double minPositionDiff = std::numeric_limits<double>::max();
            for(const auto& armor : armors) {
                auto p = getArmorPos(armor);
                if(auto positionDiff = glm::distance(predictedPosition, p); positionDiff < minPositionDiff) {
                    minPositionDiff = positionDiff;
                    candidate = { p, getArmorYaw(armor) };
                }
            }

            if(minPositionDiff < mConfig.maxMatchDist) {
                // Matching armor found
                matched = true;
                // Update EKF
                double measuredYaw = orientationToYaw(candidate.second);
                Eigen::Vector4d z(candidate.first.x, candidate.first.y, candidate.first.z, measuredYaw);
                mTrackedArmor.state = mEKF.update(z);
                HubLogger::watch("xRefRobot", z(0));
                HubLogger::watch("yRefRobot", z(1));
                HubLogger::watch("zRefRobot", z(2));
                HubLogger::watch("yawRefRobot", z(3));
                HubLogger::VisualLog(fmt::format("EKF update Matched, minPositionDiff {:.3f}", minPositionDiff));
            } else {
                // Check if there is same id armor in current frame
                HubLogger::VisualLog(fmt::format("EKF update did not matched, minPositionDiff {:.3f}, check if have another same armor", minPositionDiff));
                for(const auto& armor : armors) {
                    if(armor.id == mTrackedArmor.id) {
                        // Armor jump happens
                        matched = true;
                        handleArmorJump(getArmorPos(armor), getArmorYaw(armor));
                        break;
                    }
                }
            }
        }
        return matched;
    }

public:
    CarPredictor(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ generateKey(this) }, mTrackedArmor{ TimePoint(), Eigen::VectorXd::Zero(9), 0, -1,
                                                                               TrackingState::LOST } {
        // EKF
        // xa = x_armor, xc = x_robot_center
        // state: xc, yc, zc, yaw, v_xc, v_yc, v_zc, v_yaw, r
        // measurement: xa, ya, za, yaw
        // f - Process function
        auto f = [this](const Eigen::VectorXd& x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(4) * mDt;
            x_new(1) += x(5) * mDt;
            x_new(2) += x(6) * mDt;
            x_new(3) += x(7) * mDt;
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_f = [this](const Eigen::VectorXd&) {
            Eigen::MatrixXd f(9, 9);
            // clang-format off
            f <<  1,   0,   0,   0,   mDt, 0,   0,   0,   0,
                  0,   1,   0,   0,   0,   mDt, 0,   0,   0,
                  0,   0,   1,   0,   0,   0,   mDt, 0,   0, 
                  0,   0,   0,   1,   0,   0,   0,   mDt, 0,
                  0,   0,   0,   0,   1,   0,   0,   0,   0,
                  0,   0,   0,   0,   0,   1,   0,   0,   0,
                  0,   0,   0,   0,   0,   0,   1,   0,   0,
                  0,   0,   0,   0,   0,   0,   0,   1,   0,
                  0,   0,   0,   0,   0,   0,   0,   0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto h = [](const Eigen::VectorXd& x) {
            Eigen::VectorXd z(4);
            double xc = x(0), zc = x(2), yaw = x(3), r = x(8);
            z(0) = xc + r * cos(yaw);  // xa
            z(1) = x(1);               // ya
            z(2) = zc + r * sin(yaw);  // za
            z(3) = x(3);               // yaw
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_h = [](const Eigen::VectorXd& x) {
            Eigen::MatrixXd h(4, 9);
            double yaw = x(3), r = x(8);
            // clang-format off
            //    xc   yc   zc   yaw         vxc  vyc  vzc  vyaw r
            h <<  1,   0,   0,   -r*sin(yaw),0,   0,   0,   0,   cos(yaw),
                  0,   1,   0,   0,          0,   0,   0,   0,   0,
                  0,   0,   1,   r*cos(yaw) ,0,   0,   0,   0,   sin(yaw),
                  0,   0,   0,   1,          0,   0,   0,   0,   0;
            // clang-format on
            return h;
        };
        // Q - process noise covariance matrix
        Eigen::DiagonalMatrix<double, 9> q;
        q.diagonal() << mConfig.Q[0], mConfig.Q[1], mConfig.Q[2], mConfig.Q[3], mConfig.Q[4], mConfig.Q[5], mConfig.Q[6],
            mConfig.Q[7], mConfig.Q[8];
        // R - measurement noise covariance matrix
        Eigen::DiagonalMatrix<double, 4> r;
        r.diagonal() << mConfig.R[0], mConfig.R[1], mConfig.R[2], mConfig.R[3];
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 9> p0;
        p0.setIdentity();
        mEKF = ExtendedKalmanFilter{ f, h, j_f, j_h, q, r, p0 };
    }

    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](set_target_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(set_target_atom, TypedIdentifier<SelectedTarget>);
                ACTOR_EXCEPTION_PROBE();
                auto data = BlackBoard::instance().get<SelectedTarget>(key);

                PredictedTarget res;
                res.lastUpdate = data->lastUpdate;

                mTfGun2Robot = data->tfRobot2Gun.invTransformObj();

                if(mConfig.enablePredictor) {  // 如果使用预测功能的话，目标相对机器人的速度即为机器人坐标系下，相机所观测的速度
                    if(mTrackedArmor.trackingState == TrackingState::LOST) {
                        // init
                        if(!data->selected.has_value())
                            return;
                        init(data->selected.value());
                    } else {
                        // update
                        bool matched = update(durationCastDouble(data->lastUpdate - mTrackedArmor.lastUpdate), data->targets);

                        // Prevent radius from spreading
                        if(mTrackedArmor.state(8) < 0.2) {
                            mTrackedArmor.state(8) = 0.2;
                            mEKF.setState(mTrackedArmor.state);
                        } else if(mTrackedArmor.state(8) > 0.4) {
                            mTrackedArmor.state(8) = 0.4;
                            mEKF.setState(mTrackedArmor.state);
                        }

                        // Tracking state machine
                        handleTrackingState(matched);

                        if(mTrackedArmor.trackingState == TrackingState::TRACKING ||
                           mTrackedArmor.trackingState == TrackingState::TEMP_LOST) {
                            res.center = glm::dvec3{ mTrackedArmor.state(0), mTrackedArmor.state(1), mTrackedArmor.state(2) };
                            res.yaw = mTrackedArmor.state(3);
                            res.linearVel = glm::dvec3{ mTrackedArmor.state(4), mTrackedArmor.state(5), mTrackedArmor.state(6) };
                            res.angularVel = mTrackedArmor.state(7);
                            res.radius = { mTrackedArmor.state(8), mLastR };
                            res.y = { mTrackedArmor.state(1), mLastY };
                            sendAll(car_predict_atom_v,
                                    BlackBoard::instance().updateSync<PredictedTarget>(Identifier{ mKey.val }, res));
                        }
                    }
                    mTrackedArmor.lastUpdate = data->lastUpdate;
                    HubLogger::watch("linearVelX", res.linearVel.mVal.x);
                    HubLogger::watch("linearVelY", res.linearVel.mVal.y);
                    HubLogger::watch("linearVelZ", res.linearVel.mVal.z);
                    HubLogger::watch("angularVel", res.angularVel.mVal);
                    HubLogger::VisualLog(fmt::format(
                        "ArmorPredictor: Predictor Armor state: pose ({:.3f} {:.3f} {:.3f} {:.3f}),  linearVel ({:.3f} {:.3f} {:.3f} {:.3f}) angularVel {:.3f}",
                        mTrackedArmor.state(0), mTrackedArmor.state(1), mTrackedArmor.state(2), mTrackedArmor.state(3),
                        mTrackedArmor.state(4), mTrackedArmor.state(5), mTrackedArmor.state(6), mTrackedArmor.state(7),
                        mTrackedArmor.state(8)));
                } else {  // 如果不使用预测功能的话，将目标看作为静止状态，目标相对机器人的速度即为机器人自身速度取反
                    const auto dataPosture = BlackBoard::instance().get<PostureData>(mIMUKey);
                    if(!dataPosture.has_value() || !data->selected.has_value())
                        return;
                    res.center = getArmorPos(data->selected.value());
                    res.yaw = getArmorYaw(data->selected.value());
                    res.linearVel = -dataPosture->linearVelocityOfRobot.mVal;
                    res.angularVel = 0;
                    res.radius = { 0, 0 };
                    res.y = { res.center.mVal.y, res.center.mVal.y };
                    HubLogger::VisualLog(fmt::format("ArmorPredictor do not use predict func, position : ({:.3f} {:.3f} {:.3f}), linearVel: ({:.3f} {:.3f} {:.3f})",
                                                     res.center.mVal.x, res.center.mVal.y, res.center.mVal.z,
                                                     res.linearVel.mVal.x, res.linearVel.mVal.y, res.linearVel.mVal.z));
                    sendAll(car_predict_atom_v, BlackBoard::instance().updateSync<PredictedTarget>(Identifier{ mKey.val }, res));
                }
            },
            [this](update_posture_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(update_posture_atom, TypedIdentifier<PostureData>);
                mIMUKey = key;
            }

        };
    }
};

HUB_REGISTER_CLASS(CarPredictor);
