#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "ExceptionProbe.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"
#include "Utility.hpp"
#include <cstdint>
#include <utility>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <eigen3/Eigen/Dense>
#include <fmt/format.h>
#include <magic_enum.hpp>

constexpr double maxDeltaTime = 0.2;
constexpr int32_t maxCntWrongData = 5;

constexpr double maxJumpXDist = 0.5;
constexpr double maxJumpYDist = 0.5;
constexpr double maxJumpZDist = 0.5;

constexpr double maxXVel = 3.0;
constexpr double maxYVel = 3.0;
constexpr double maxZVel = 1.0;

constexpr double staticVelThreshold = 0.1;
constexpr double staticPosThreshold = 0.01;
constexpr Duration maxWaitingTime = 50ms;
constexpr double maxJumpTheta = glm::radians<double>(10);

struct ArmorPredictorSettings final {
    bool enablePredictor;
};

template <class Inspector>
bool inspect(Inspector& f, ArmorPredictorSettings& x) {
    return f.object(x).fields(f.field("enablePredictor", x.enablePredictor));
}

class ArmorPredictor final
    : public HubHelper<caf::event_based_actor, ArmorPredictorSettings, predict_success_atom, outpost_predict_success_atom> {
    Identifier mKey, mIMUKey;
    GroupMask mGroupMask;

    std::queue<std::pair<TimePoint, Vector<UnitType::Distance, FrameOfRef::Robot>>> lastTwoPosition;
    std::optional<Scalar<UnitType::Angle>> lastTheta;

    bool mInitFlag = false;
    int32_t mCntWrongData = 0;
    glm::dvec3 mLastPos;
    TimePoint mLastTimePoint;
    glm::dvec3 mPredictedVel;
    Eigen::VectorXd mX;  // State vector(Position & Velocity)
    Eigen::MatrixXd mF;  // State tfRobot2Gun mat
    Eigen::MatrixXd mP;  // State covariance mat
    Eigen::MatrixXd mQ;  // Process covariance mat
    Eigen::MatrixXd mH;  // Measurement mat
    Eigen::MatrixXd mR;  // Measurement covariance mat

    void setX(const glm::dvec3& measuredPos) {
        Eigen::VectorXd initX(6, 1);
        initX << measuredPos.x, measuredPos.y, measuredPos.z, 0.0, 0.0, 0.0;
        mX = initX;
    }

    // 检查数据是否跳变，跳变数据连续出现五次，则重新初始化滤波器
    glm::dvec3 filterWrongData(const glm::dvec3& measuredPos) {
        glm::dvec3 filterRes = measuredPos;
        bool isWrongData = false;
        if(mCntWrongData < maxCntWrongData) {  // 如果跳变数据小于maxCntWrongData,则继续计数
            if(std::fabs(measuredPos.x - mLastPos.x) > maxJumpXDist) {
                isWrongData = true;
                filterRes.x = mLastPos.x;
            }
            if(std::fabs(measuredPos.y - mLastPos.y) > maxJumpYDist) {
                isWrongData = true;
                filterRes.y = mLastPos.y;
            }
            if(std::fabs(measuredPos.z - mLastPos.z) > maxJumpXDist) {
                isWrongData = true;
                filterRes.z = mLastPos.z;
            }
        } else {  // 跳变数据个数超过maxCntWrongData, 则重新初始化滤波器
            mInitFlag = false;
            mCntWrongData = 0;
        }

        if(isWrongData) {  // 无跳变数据出现,为正常数据，对目标进行速度预测
            mCntWrongData = 0;
        } else {
            ++mCntWrongData;  // 跳变数据出现，进行计数
        }
        return filterRes;
    }

    void initialKalmanFilter(const glm::dvec3& measuredPos, const TimePoint& curTimePoint) {
        mX.resize(6);
        mF.setIdentity(6, 6);
        mP.setIdentity(6, 6);
        mQ.setIdentity(6, 6);
        mH.resize(3, 6);
        mH << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,  //
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,    //
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0;    //
        mR.resize(3, 3);
        mR << 0.01, 0.0, 0.0,  //
            0.0, 0.01, 0.0,    //
            0.0, 0.0, 0.01;

        mLastPos = measuredPos;
        mLastTimePoint = curTimePoint;
        setX(measuredPos);
        mInitFlag = true;
    }

    void Prediction() {
        mX = mF * mX;
        mP = mF * mP * mF.transpose() + mQ;
    }

    void UpdateMeasurement(const Eigen::VectorXd z) {
        const auto y = z - mH * mX;  // Measure
        const auto S = mH * mP * mH.transpose() + mR;
        const auto K = mP * mH.transpose() * S.inverse();  // Kalman Gain
        mX = mX + (K * y);                                 // Optimal estimate
        const auto I = Eigen::MatrixXd::Identity(mX.size(), mX.size());
        mP = (I - K * mH) * mP;
    }

    void KmFilter(const glm::dvec3& pos, double dt) {
        Eigen::MatrixXd inputF(6, 6);
        inputF << 1.0, 0.0, 0.0, dt, 0.0, 0.0,  //
            0.0, 1.0, 0.0, 0.0, dt, 0.0,        //
            0.0, 0.0, 1.0, 0.0, 0.0, dt,        //
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,       //
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,       //
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;       //
        mF << inputF;

        Prediction();
        Eigen::VectorXd measuredZ(3, 1);
        measuredZ << pos.x, pos.y, pos.z;
        UpdateMeasurement(measuredZ);

        mPredictedVel = { mX(3), mX(4), mX(5) };
        if(mPredictedVel.x > maxXVel) {  // 限幅滤波
            mPredictedVel.x = maxXVel;
        }
        if(mPredictedVel.y > maxYVel) {
            mPredictedVel.y = maxYVel;
        }
        if(mPredictedVel.z > maxZVel) {
            mPredictedVel.z = maxZVel;
        }
    }

    void runFilter(const glm::dvec3& measuredPos, const TimePoint& curTimePoint) {
        if(!mInitFlag) {
            initialKalmanFilter(measuredPos, curTimePoint);
            return;
        }

        double deltaTime =
            static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(curTimePoint - mLastTimePoint).count()) /
            1e6;
        if(deltaTime > maxDeltaTime) {
            initialKalmanFilter(measuredPos, curTimePoint);
            return;
        }
        const auto filterPos = filterWrongData(measuredPos);
        if(!mInitFlag) {
            initialKalmanFilter(measuredPos, curTimePoint);
            return;
        }
        KmFilter(filterPos, deltaTime);
    }

    // a,b,c mustn't be on the same line or on the same point
    glm::dvec3 circleCenter(glm::dvec3 a, glm::dvec3 b, glm::dvec3 c) {
        double a1, b1, c1, d1;
        double a2, b2, c2, d2;
        double a3, b3, c3, d3;

        double la, lb, lc, l;

        glm::dvec3 res;

        la = a.x * a.x + a.y * a.y + a.z * a.z;
        lb = b.x * b.x + b.y * b.y + b.z * b.z;
        lc = c.x * c.x + c.y * c.y + c.z * c.z;

        a1 = (a.y * b.z - b.y * a.z - a.y * c.z + c.y * a.z + b.y * c.z - c.y * b.z);
        b1 = -(a.x * b.z - b.x * a.z - a.x * c.z + c.x * a.z + b.x * c.z - c.x * b.z);
        c1 = (a.x * b.y - b.x * a.y - a.x * c.y + c.x * a.y + b.x * c.y - c.x * b.y);
        d1 = a.x * b.z * c.y + a.y * b.x * c.z + a.z * b.y * c.x - a.x * b.y * c.z - a.y * b.z * c.x - a.z * b.x * c.y;

        a2 = 2 * (b.x - a.x);
        b2 = 2 * (b.y - a.y);
        c2 = 2 * (b.z - a.z);
        d2 = la - lb;

        a3 = 2 * (c.x - a.x);
        b3 = 2 * (c.y - a.y);
        c3 = 2 * (c.z - a.z);
        d3 = la - lc;

        l = a1 * b2 * c3 + a2 * b3 * c1 + a3 * b1 * c2 - a1 * b3 * c2 - a2 * b1 * c3 - a3 * b2 * c1;

        res.x = -(b1 * c2 * d3 + b2 * c3 * d1 + b3 * c1 * d2 - b1 * c3 * d2 - b2 * c1 * d3 - b3 * c2 * d1) / l;
        res.y = (a1 * c2 * d3 + a2 * c3 * d1 + a3 * c1 * d2 - a1 * c3 * d2 - a2 * c1 * d3 - a3 * c2 * d1) / l;
        res.z = -(a1 * b2 * d3 + a2 * b3 * d1 + a3 * b1 * d2 - a1 * b3 * d2 - a2 * b1 * d3 - a3 * b2 * d1) / l;

        return res;
    }

public:
    ArmorPredictor(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}

    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](set_target_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(set_target_atom, TypedIdentifier<SelectedTarget>);
                ACTOR_EXCEPTION_PROBE();

                auto data = BlackBoard::instance().get<SelectedTarget>(key);
                const auto dataPosture = BlackBoard::instance().get<PostureData>(mIMUKey);
                if(!(data->selected.has_value() && dataPosture.has_value() && data->tfRobot2Gun.has_value()))
                    return;
                HubLogger::watch("armor type", magic_enum::enum_name(data->selected->type));

                PredictedTarget res;
                res.lastUpdate = data->lastUpdate;

                Vector<UnitType::Distance, FrameOfRef::Gun> posOfRefGun(data->selected->center.mVal);
                Vector<UnitType::Distance, FrameOfRef::Robot> posRefRobot = data->tfRobot2Gun->invTransform(posOfRefGun);
                res.position = posRefRobot;

                if(mConfig.enablePredictor) {  // 如果使用预测功能的话，目标相对机器人的速度即为机器人坐标系下，相机所观测的速度
                    glm::dvec3 measuredPos = posRefRobot.mVal;
                    runFilter(measuredPos, data.value().lastUpdate);
                    res.velocity = mPredictedVel;
                } else {  // 如果不使用预测功能的话，将目标看作为静止状态，目标相对机器人的速度即为机器人自身速度取反
                    res.velocity = -dataPosture->linearVelocityOfRobot.mVal;
                }

                sendAll(predict_success_atom_v, BlackBoard::instance().updateSync<PredictedTarget>(Identifier{ mKey.val }, res));
            },
            [this](set_outpost_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(set_target_atom, TypedIdentifier<SelectedTarget>);
                ACTOR_EXCEPTION_PROBE();

                auto data = BlackBoard::instance().get<SelectedTarget>(key);
                const auto dataPosture = BlackBoard::instance().get<PostureData>(mIMUKey);
                if(!(data->selected.has_value() && dataPosture.has_value() && data->tfRobot2Gun.has_value()))
                    return;
                if(!(std::fabs(dataPosture->linearVelocityOfRobot.mVal.x) <= staticVelThreshold &&
                     std::fabs(dataPosture->linearVelocityOfRobot.mVal.y) <= staticVelThreshold &&
                     std::fabs(dataPosture->linearVelocityOfRobot.mVal.z) <= staticVelThreshold)) {
                    logInfo("AromorPredictor: out of static vel threshold");
                    lastTwoPosition.pop();
                    lastTwoPosition.pop();
                    lastTheta = std::nullopt;
                    return;
                }
                HubLogger::watch("armor type", magic_enum::enum_name(data->selected->type));

                PredictedOutpost res;
                res.lastUpdate = data->lastUpdate;

                Vector<UnitType::Distance, FrameOfRef::Gun> posOfRefGun(data->selected->center.mVal);
                Vector<UnitType::Distance, FrameOfRef::Robot> posRefRobot = data->tfRobot2Gun->invTransform(posOfRefGun);

                if(mConfig.enablePredictor) {  // 如果使用预测功能的话，修正旋转中心及当前位置
                } else {                       // 如果不使用预测功能的话，不修正位置
                }
                {  //下面是不使用预测的
                    static int stopTimes = 0;
                    static bool stopPrinted = false;
                    static std::optional<int> direction;
                    if(stopTimes > 5) {
                        if(!stopPrinted) {
                            logInfo("stop");
                            stopPrinted = true;
                        }
                        res.angularVelocity = 0;
                        res.theta = glm::radians(90.0);
                        res.centerOfOutpost = posRefRobot;
                        res.centerOfOutpost.mVal.z -= radiusOfOutpost;
                    } else {
                        if(lastTwoPosition.size() == 0) {
                            lastTwoPosition.push(std::make_pair(data->lastUpdate, posRefRobot));
                            return;
                        }
                        if(posRefRobot == lastTwoPosition.back().second || posRefRobot == lastTwoPosition.front().second) {
                            stopTimes += 1;
                            return;
                        }
                        if(lastTwoPosition.size() == 1) {
                            lastTwoPosition.push(std::make_pair(data->lastUpdate, posRefRobot));
                            return;
                        }
                        if(data->lastUpdate - lastTwoPosition.front().first > maxWaitingTime) {
                            if(glm::distance(posRefRobot.mVal, lastTwoPosition.front().second.mVal) > staticPosThreshold) {
                                logInfo("remove stop");
                                stopPrinted = false;
                                stopTimes = 0;
                            } else {
                                stopTimes += 1;
                            }
                            lastTwoPosition.pop();
                            lastTwoPosition.push(std::make_pair(data->lastUpdate, posRefRobot));
                            lastTheta = std::nullopt;
                            return;
                        }
                        res.centerOfOutpost = circleCenter(lastTwoPosition.front().second.mVal,
                                                           lastTwoPosition.back().second.mVal, posRefRobot.mVal);
                        {
                            double x = posRefRobot.mVal.x - res.centerOfOutpost.mVal.x;
                            double z = posRefRobot.mVal.z - res.centerOfOutpost.mVal.z;
                            double l = glm::sqrt(x * x + z * z);
                            res.theta = glm::acos(x / l);
                        }
                        if(!lastTheta.has_value()) {
                            double x = lastTwoPosition.back().second.mVal.x - res.centerOfOutpost.mVal.x;
                            double z = lastTwoPosition.back().second.mVal.z - res.centerOfOutpost.mVal.z;
                            double l = glm::sqrt(x * x + z * z);
                            lastTheta = glm::acos(x / l);
                        }
                        if(!direction.has_value()) {
                            if(std::fabs((lastTheta.value() - res.theta).mVal) > maxJumpTheta)
                                direction = (lastTheta.value() > res.theta ? 1 : -1);
                            else
                                direction = (res.theta > lastTheta.value() ? 1 : -1);
                        }
                        if(std::fabs((lastTheta.value() - res.theta).mVal) > maxJumpTheta)
                            lastTheta.value().mVal -= direction.value() * glm::radians<double>(120);
                        res.angularVelocity = (res.theta - lastTheta.value()) /
                            Scalar<UnitType::Time>{ static_cast<double>(
                                                        (data->lastUpdate - lastTwoPosition.back().first).count()) /
                                                    Clock::period::den * Clock::period::num };
                        lastTheta = res.theta;
                        lastTwoPosition.pop();
                        lastTwoPosition.push(std::make_pair(data->lastUpdate, posRefRobot));
                    }
                }

                sendAll(outpost_predict_success_atom_v,
                        BlackBoard::instance().updateSync<PredictedOutpost>(Identifier{ mKey.val }, res));
            },
            [this](update_posture_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(update_posture_atom, TypedIdentifier<PostureData>);
                mIMUKey = key;
            }
        };
    }
};

HUB_REGISTER_CLASS(ArmorPredictor);
