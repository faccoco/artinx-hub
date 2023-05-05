// https://github.com/chenjunnn/rm_auto_aim

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
#include <eigen3/Eigen/Dense>
#include <magic_enum.hpp>

struct ArmorPredictorSettings final {
    bool enablePredictor;
    double maxMatchDistance;
    int trackingThreshold;
    int lostThreshold;
    std::vector<double> Q;  // Process covariance matrix
    std::vector<double> R;  // Measurement covariance mat
};

template <class Inspector>
bool inspect(Inspector& f, ArmorPredictorSettings& x) {
    return f.object(x).fields(
        f.field("enablePredictor", x.enablePredictor), f.field("maxMatchDistance", x.maxMatchDistance).fallback(0.2),
        f.field("trackingThreshold", x.trackingThreshold).fallback(5), f.field("lostThreshold", x.lostThreshold).fallback(5),
        f.field("Q", x.Q).invariant([](auto& c) { return c.size() == 9; }).fallback(std::vector<double>(9, 0)),
        f.field("R", x.R).invariant([](auto& c) { return c.size() == 4; }).fallback(std::vector<double>(4, 0)));
}

class ArmorPredictor final : public HubHelper<caf::event_based_actor, ArmorPredictorSettings, predict_success_atom> {
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
    double mLastY, mLastR;
    int mDetectCount, mLostCount;
    double mDt;

    Transform<FrameOfRef::Gun, FrameOfRef::Robot, true> mTfGun2Robot;

    glm::dvec3 getArmorPos(const DetectedTarget& armor) {
        return mTfGun2Robot(Vector<UnitType::Distance, FrameOfRef::Gun>(armor.center.mVal)).mVal;
    }

    double getArmorYaw(const DetectedTarget& armor) {
        auto rvec = mTfGun2Robot(armor.rvec).mVal;
        double theta = glm::length(rvec);
        glm::dvec3 vec = rvec / theta;
        double s = sin(theta), c = cos(theta);

        return -atan2(s * vec.y + (1 - c) * vec.x * vec.z, c + (1 - c) * vec.z * vec.z);
    }

    double orientationToYaw(double yaw) {
        mTrackedArmor.yaw = mTrackedArmor.yaw + shortestAngularDistance(yaw, mTrackedArmor.yaw);
        return mTrackedArmor.yaw;
    }

    glm::dvec3 getArmorPositionFromState(const Eigen::VectorXd& x) {
        // Calculate predicted position of the current armor
        double xc = x(0), ya = x(1), zc = x(2);
        double yaw = x(3), r = x(8);
        double xa = xc + r * cos(yaw);
        double za = zc + r * sin(yaw);
        return glm::dvec3{ xa, ya, za };
    }

    void handleArmorJump(const glm::dvec3& targetPos, double targetYaw) {
        double yaw = orientationToYaw(targetYaw);
        if(abs(yaw - mTrackedArmor.state(3)) > 0.4) {
            mLastY = mTrackedArmor.state(1);
            mTrackedArmor.state(1) = targetPos.y;
            mTrackedArmor.state(3) = yaw;
            std::swap(mTrackedArmor.state(8), mLastR);
            logInfo("ArmorPredictor: Armor jump!");
        }
        if(glm::distance(targetPos, getArmorPositionFromState(mTrackedArmor.state)) > mConfig.maxMatchDistance) {
            mTrackedArmor.state(0) = targetPos.x - mTrackedArmor.state(8) * cos(yaw);
            mTrackedArmor.state(2) = targetPos.z - mTrackedArmor.state(8) * sin(yaw);
            mTrackedArmor.state(4) = 0;
            mTrackedArmor.state(5) = 0;
            mTrackedArmor.state(6) = 0;
            logInfo("ArmorPredictor: State wrong!");
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
        logInfo(fmt::format("ArmorPredictor: tracking state: {}", magic_enum::enum_name(mTrackedArmor.trackingState)));
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
        mTrackedArmor.id = armor.id;
        mTrackedArmor.trackingState = TrackingState::DETECTING;
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
            auto predictedPosition = getArmorPositionFromState(ekfPrediction);
            // Difference of the current armor position and tracked armor's predicted position
            double minPositionDiff = std::numeric_limits<double>::max();
            for(const auto& armor : armors) {
                auto p = getArmorPos(armor);
                if(auto positionDiff = glm::distance(predictedPosition, p); positionDiff < minPositionDiff) {
                    minPositionDiff = positionDiff;
                    candidate = { p, getArmorYaw(armor) };
                }
            }

            if(minPositionDiff < mConfig.maxMatchDistance) {
                // Matching armor found
                matched = true;
                // Update EKF
                double measuredYaw = orientationToYaw(candidate.second);
                Eigen::Vector4d z(candidate.first.x, candidate.first.y, candidate.first.z, measuredYaw);
                mTrackedArmor.state = mEKF.update(z);
                logInfo(fmt::format("ArmorPredictor: update: {:.3f} {:.3f} {:.3f} {:.3f}", z(0), z(1), z(2), z(3)));
                HubLogger::watch("update yaw", glm::degrees(normalizeAngle(z(3))));
            } else {
                // Check if there is same id armor in current frame
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
    ArmorPredictor(caf::actor_config& base, const HubConfig& config)
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
                        if(data->selected.empty())
                            return;
                        init(data->selected[0]);
                    } else {
                        // update
                        bool matched = update(durationCastDouble(data->lastUpdate - mTrackedArmor.lastUpdate), data->selected);

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
                            res.theta = mTrackedArmor.state(3);
                            res.lVel = glm::dvec3{ mTrackedArmor.state(4), mTrackedArmor.state(5), mTrackedArmor.state(6) };
                            res.aVel = mTrackedArmor.state(7);
                            res.radius = { mTrackedArmor.state(8), mLastR };
                            res.y = { mTrackedArmor.state(1), mLastY };
                            sendAll(predict_success_atom_v,
                                    BlackBoard::instance().updateSync<PredictedTarget>(Identifier{ mKey.val }, res));
                        }
                    }
                    logInfo(fmt::format("ArmorPredictor: state:{:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}",
                                        mTrackedArmor.state(0), mTrackedArmor.state(1), mTrackedArmor.state(2),
                                        mTrackedArmor.state(3), mTrackedArmor.state(4), mTrackedArmor.state(5),
                                        mTrackedArmor.state(6), mTrackedArmor.state(7), mTrackedArmor.state(8)));
                    mTrackedArmor.lastUpdate = data->lastUpdate;
                } else {  // 如果不使用预测功能的话，将目标看作为静止状态，目标相对机器人的速度即为机器人自身速度取反
                    const auto dataPosture = BlackBoard::instance().get<PostureData>(mIMUKey);
                    if(!dataPosture.has_value() || data->selected.empty())
                        return;
                    res.center = getArmorPos(data->selected[0]);
                    res.theta = getArmorYaw(data->selected[0]);
                    res.lVel = -dataPosture->linearVelocityOfRobot.mVal;
                    res.aVel = 0;
                    res.radius = { 0, 0 };
                    res.y = { res.center.mVal.y, res.center.mVal.y };
                    logInfo(fmt::format("lVel:{:.3f} {:.3f} {:.3f}", res.lVel.mVal.x, res.lVel.mVal.y, res.lVel.mVal.z));
                    sendAll(predict_success_atom_v,
                            BlackBoard::instance().updateSync<PredictedTarget>(Identifier{ mKey.val }, res));
                }
            },
            [this](update_posture_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(update_posture_atom, TypedIdentifier<PostureData>);
                mIMUKey = key;
            }

        };
    }
};

HUB_REGISTER_CLASS(ArmorPredictor);
