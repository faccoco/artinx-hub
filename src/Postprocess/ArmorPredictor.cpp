#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <eigen3/Eigen/Dense>
#include <fmt/format.h>
#include <magic_enum.hpp>

constexpr double maxDeltaTime = 0.2;

struct ArmorPredictorSettings final {
    bool enablePredictor;
    double maxResidual;     // max residual for chi-square test
    std::vector<double> P;  // State covariance matrix
    std::vector<double> Q;  // Process covariance matrix
    std::vector<double> R;  // Measurement covariance mat
};

template <class Inspector>
bool inspect(Inspector& f, ArmorPredictorSettings& x) {
    return f.object(x).fields(f.field("enablePredictor", x.enablePredictor), f.field("maxResidual", x.maxResidual).fallback(0),
                              f.field("P", x.P).invariant([](auto& c) { return c.size() == 6; }).fallback(std::vector<double>(0)),
                              f.field("Q", x.Q).invariant([](auto& c) { return c.size() == 6; }).fallback(std::vector<double>(0)),
                              f.field("R", x.R).invariant([](auto& c) { return c.size() == 3; }).fallback(std::vector<double>(0)));
}

/*   Kalman Filter:
X(k) = F(k) * X(k-1) + B(k) * u(k-1) + w   w~N(0, Q)
Z(k) = H(k) * X(k) + v                     v~N(0, R)

Step1:先验估计，通过状态空间方程进行预测
x_(k|k-1) = F(k)x_(k-1|k-1)
Step2:先验状态误差协方差矩阵更新
P(k|k-1) = F(k) * P(k-1|k-1) F(k)' + Q(k-1)
Step3:卡尔曼增益更新
K(k) = P(k|k-1) H(k)'(H(k) * P(k|k-1) * H(k)' + R)^(-1)
Step4:后验估计，通过测量信息修正预测结果:
x_(k|k) = x_(k|k-1) + K(k) * (z(k) - H(k) * x_(k|k-1))
Step5:后验误差协方差矩阵更新:
P(k|k) = (I - K(k) * H(k)) * P(k|k-1)
*/
class ArmorPredictor final : public HubHelper<caf::event_based_actor, ArmorPredictorSettings, predict_success_atom> {
    Identifier mKey, mIMUKey;

    bool mInitFlag = false;
    glm::dvec3 mLastPos;
    TimePoint mLastTimePoint;
    glm::dvec3 mPredictedVel;
    Eigen::VectorXd mX;  // State vector(Position & Velocity)
    Eigen::MatrixXd mF;  // State transform mat
    Eigen::MatrixXd mP;  // State covariance mat
    Eigen::MatrixXd mQ;  // Process covariance mat
    Eigen::MatrixXd mH;  // Measurement mat
    Eigen::MatrixXd mR;  // Measurement covariance mat

    void initialKalmanFilter(const glm::dvec3& measuredPos, const TimePoint& curTimePoint) {
        mX.resize(6);
        mF.setIdentity(6, 6);
        mP.setIdentity(6, 6);
        mQ.setIdentity(6, 6);

        for(int i = 0; i < 6; ++i) {
            mP(i, i) = mConfig.P[i];
            mQ(i, i) = mConfig.Q[i];
        }

        mH.resize(3, 6);
        mH << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,  //
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,    //
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0;    //

        mR.setIdentity(3, 3);
        for(int i = 0; i < 3; ++i) {
            mR(i, i) = mConfig.R[i];
        }

        mLastTimePoint = curTimePoint;
        mX << measuredPos.x, measuredPos.y, measuredPos.z, 0.0, 0.0, 0.0;
        mPredictedVel = { 0, 0, 0 };
        mInitFlag = true;
    }

    void Prediction() {
        mX = mF * mX;
        mP = mF * mP * mF.transpose() + mQ;
    }

    void UpdateMeasurement(const Eigen::VectorXd z) {
        const auto y = z - mH * mX;  // Measure
        const auto invS = (mH * mP * mH.transpose() + mR).inverse();

        // 卡方检验判断目标是否切换
        const auto residual = y.transpose() * invS * y;
        if(residual > mConfig.maxResidual) {
            // logInfo("Target changed!");
            mInitFlag = false;
            return;
        }

        const auto K = mP * mH.transpose() * invS;  // Kalman Gain
        mX = mX + (K * y);                          // Optimal estimate
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

        KmFilter(measuredPos, deltaTime);
        if(!mInitFlag) {
            initialKalmanFilter(measuredPos, curTimePoint);
            return;
        }
        mLastTimePoint = curTimePoint;
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
                //                logInfo(fmt::format("selected: {}, posture: {}, tfRobot2Gun: {}", data->selected.has_value(),
                //                dataPosture.has_value(), data->tfRobot2Gun.has_value()));
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
                    // logInfo(fmt::format("{}, {}, {}", measuredPos.z, measuredPos.y, measuredPos.x));
                } else {  // 如果不使用预测功能的话，将目标看作为静止状态，目标相对机器人的速度即为机器人自身速度取反
                    res.velocity = -dataPosture->linearVelocityOfRobot.mVal;
                }
                //                logInfo("Predictor works well");
                sendAll(predict_success_atom_v, BlackBoard::instance().updateSync<PredictedTarget>(Identifier{ mKey.val }, res));
            },
            [this](update_posture_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(update_posture_atom, TypedIdentifier<PostureData>);
                mIMUKey = key;
            }

        };
    }
};

HUB_REGISTER_CLASS(ArmorPredictor);
