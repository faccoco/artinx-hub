#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedEnergyFan.hpp"
#include "EKF.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <caf/event_based_actor.hpp>
#include <ceres/ceres.h>
#include <cmath>
#include <fmt/format.h>
#include <glm/fwd.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include "SuppressWarningEnd.hpp"

static constexpr double minPositionDiff = 1.8;
static constexpr double minThetaDiff = 0.7;  // < pi/3
static constexpr double diffTThresh = 0.005;

static constexpr double fanLen = 0.705;
static constexpr double longRuneArmorWidth = 0.330;
static constexpr double shortRuneArmorWidth = 0.305;
static constexpr double runeArmorHeight = 0.095;

struct EnergyPredictorSettings final {
    uint8_t fanQueueLength;
    uint8_t setMode;
    double Qw;
    double Qtheta;
    double Qxyz;
    double Qyaw;
    double Rtheta;
    double Rxyz;
    double Ryaw;
    double delay;
    int lostCnt;
};

template <class Inspector>
bool inspect(Inspector& f, EnergyPredictorSettings& x) {
    return f.object(x).fields(f.field("fanQueueLength", x.fanQueueLength), f.field("setMode", x.setMode).fallback(0),
                              f.field("Qw", x.Qw), f.field("Qtheta", x.Qtheta), f.field("Qxyz", x.Qxyz), f.field("Qyaw", x.Qyaw),
                              f.field("Rtheta", x.Rtheta), f.field("Rxyz", x.Rxyz), f.field("Ryaw", x.Ryaw),
                              f.field("delay", x.delay).fallback(0.0), f.field("lostCnt", x.lostCnt).fallback(20));
}

struct CurveFittingCost {
    CurveFittingCost(double t_, double theta_) : t(t_), theta(theta_) {}

    template <typename T>
    bool operator()(const T* params, T* residual) const {
        //-a/w * cost(wt + phi) + (2.090 - a) * t + C
        residual[0] = T(theta) -
            (-params[0] / params[1] * ceres::cos(params[1] * T(t) + params[2]) + (2.090 - params[0]) * T(t) + params[3]);
        return true;
    }

    const double t, theta;
};

class EnergyPredictor final : public HubHelper<caf::event_based_actor, EnergyPredictorSettings, set_target_info_atom> {

    Identifier mKey;
    TaskMode mMode;
    int mDirSum = 0, mDirection = 0;  // mDirection (-1 Clockwise, 1 anti-clockwise)
    constexpr static int mLostCountThresh = 10;
    double mParameters[4]{ 0.780, 1.884, 0.0, 0.0 };
    double mDt;
    int mLostCount = 0;
    TimePoint mLastHit{};
    std::deque<std::tuple<TimePoint, double, double, double>> mThetaInfos;  //(time, theta, dt, dTheta)
    ExtendedKalmanFilter mFilter;

    enum class FanTrackingState { LOST, TRACKING };

    struct TrackedFan {
        TimePoint lastUpdate;
        Eigen::VectorXd state;  // w, theta, xr, yr, zr, yaw
        FanTrackingState trackSate;
    } mTrackFan;
    Transform<FrameOfRef::Camera, FrameOfRef::Robot, true> mTfCamera2Robot;

    void reset() {
        mThetaInfos.clear();
        mTrackFan.trackSate = FanTrackingState ::LOST;
        mDirSum = 0;
        mTrackFan.state(2) = 0;  // theta = 0
        mTrackFan.state(6) = 0;  // yaw = 0
    }

    void init(const Eigen::VectorXd& fanPos) {
        mTrackFan.state = getRLabelPosFromFanPos(fanPos);
        double initAVel = 1.0472;
        if(mMode == TaskMode::BigRune)
            initAVel = 1.0;  //
        mTrackFan.state(0) = 0.0;
        mTrackFan.state(1) = initAVel;
        mFilter.setState(mTrackFan.state);
        logInfo("EnergyPredictor: Init EKF!");
    }

    Eigen::VectorXd getFanPosFromState(const Eigen::VectorXd& X) {
        Eigen::VectorXd fanPos(5);
        double rLabelX = X(3), rLabelY = X(4), rLabeZ = X(5);
        double theta = X(2), yaw = X(6);
        fanPos(0) = theta;
        fanPos(1) = rLabelX + fanLen * std::cos(theta) * std::cos(yaw);
        fanPos(2) = rLabelY + fanLen * std::sin(theta);
        fanPos(3) = rLabeZ + fanLen * std::cos(theta) * std::sin(yaw);
        fanPos(4) = yaw;
        return fanPos;
    }

    Eigen::VectorXd getRLabelPosFromFanPos(const Eigen::VectorXd& fanPos) {
        Eigen::VectorXd rLabelPos(7);
        double theta = fanPos(0), yaw = fanPos(4);
        rLabelPos(2) = theta;
        rLabelPos(3) = fanPos(1) - fanLen * std::cos(theta) * std::cos(yaw);  // xr
        rLabelPos(4) = fanPos(2) - fanLen * std::sin(theta);
        rLabelPos(5) = fanPos(3) - fanLen * std::cos(theta) * std::sin(yaw);
        rLabelPos(6) = yaw;
        return rLabelPos;
    }

    bool update(double dt, const Eigen::VectorXd& fanPosition) {
        mDt = dt;
        auto ekfPredict = mFilter.predict();
        mTrackFan.state = ekfPredict;
        auto fanPredict = getFanPosFromState(ekfPredict);
        double positionDiff = (fanPosition.tail(4) - fanPredict.tail(4)).norm();
        double thetaDiff = std::abs(fanPosition(0) - fanPredict(0));
        HubLogger::visualLog(fmt::format("predictedPos: w theta x y z yaw {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}",
                                         ekfPredict(1), ekfPredict(2), ekfPredict(3), ekfPredict(4), ekfPredict(5),
                                         ekfPredict(6)));
        logInfo(fmt::format("Energy Predictor: positionDiff {:.3f} and thetaDiff {:.3f}", positionDiff, thetaDiff));
        if(positionDiff < minPositionDiff) {
            if(thetaDiff < minThetaDiff) {
                mTrackFan.state = mFilter.update(fanPosition);
                logInfo("TrackFan Matched");
            } else {
                double t = mTrackFan.state(0), w = mTrackFan.state(1);
                mTrackFan.state = getRLabelPosFromFanPos(fanPosition);
                mTrackFan.state(0) = t;
                mTrackFan.state(1) = w;
                mTrackFan.state(2) = normalizeAngle(mTrackFan.state(2));
                mFilter.setState(mTrackFan.state);
                mThetaInfos.clear();
                HubLogger::visualLog(fmt::format("Fan Jumped positionDiff {:.3f} and thetaDiff {:.3f}", positionDiff, thetaDiff));
            }
            return true;
        } else {
            logInfo("TrackFan do not matched!");
            return false;
        }
    }

    void changeTrackingState(bool matched) {
        if(matched) {
            mTrackFan.trackSate = FanTrackingState::TRACKING;
            mLostCount = 0;
        } else {
            if(mTrackFan.trackSate == FanTrackingState::TRACKING) {
                mLostCount++;
                if(mLostCount >= mLostCountThresh) {
                    mTrackFan.trackSate = FanTrackingState ::LOST;
                    mLostCount = 0;
                }
            }
        }
    }

    double clcBigRuneTheta(double t0, double dt) {
        double a = mParameters[0], w = mParameters[1], p = mParameters[2], c = mParameters[3];
        return -a / w * std::cos(w * (t0 + dt) + p) + (2.090 - a) * (t0 + dt) + c;
    }

    double clcTheta(const cv::Point2f& rCenter, const cv::Point2f& fanCenter) {
        double dy = rCenter.y - fanCenter.y, dx = fanCenter.x - rCenter.x;
        return std::atan2(dy, dx);
    }

    cv::Point2f clcFanImgCenter(const std::vector<cv::Point2f>& imagePoints) {
        return { (imagePoints[0].x + imagePoints[1].x + imagePoints[2].x + imagePoints[3].x) / 4,
                 (imagePoints[0].y + imagePoints[1].y + imagePoints[2].y + imagePoints[3].y) / 4 };
    }

    std::tuple<bool, glm::dvec3, double, glm::dmat4> pnpSolver(const std::vector<cv::Point2f>& keyPoints,
                                                               const CameraInfo& cameraInfo) {
        const std::vector<cv::Point3d> mFanObjectPoints = { { +longRuneArmorWidth / 2, +runeArmorHeight / 2, 0.0 },
                                                            { -longRuneArmorWidth / 2, +runeArmorHeight / 2, 0.0 },
                                                            { +shortRuneArmorWidth / 2, -runeArmorHeight / 2, 0.0 },
                                                            { -shortRuneArmorWidth / 2, -runeArmorHeight / 2, 0.0 } };
        std::vector<cv::Point2f> imagePoints(4);
        for(int i = 0; i < 4; ++i) {
            imagePoints[i].x = keyPoints[i].x;
            imagePoints[i].y = keyPoints[i].y;
        }

        cv::Mat rvec, tvec;
        const auto pnpRes = cv::solvePnP(mFanObjectPoints, imagePoints, cameraInfo.cameraMatrix, cameraInfo.distCoefficients,
                                         rvec, tvec, false, cv::SOLVEPNP_IPPE);

        glm::dvec3 fanCenter = { tvec.at<double>(0, 0), -tvec.at<double>(1, 0), -tvec.at<double>(2, 0) };
        glm::dvec3 rvecRefCam = { rvec.at<double>(0, 0), -rvec.at<double>(1, 0), -rvec.at<double>(2, 0) };

        if(!pnpRes || std::isnan(fanCenter.x) || std::isnan(fanCenter.y) || std::isnan(fanCenter.z))
            return std::tuple<bool, glm::dvec3, double, glm::dmat4>{ false, 0, 0, 0 };

        double angle = glm::length(rvecRefCam);
        auto axis = rvecRefCam / angle;
        auto rmat = glm::mat4_cast(glm::angleAxis(-angle, axis));

        double theta = clcTheta(keyPoints[4], clcFanImgCenter(imagePoints));
        return std::make_tuple(pnpRes, fanCenter, theta, rmat);
    }

    void saveThetaInfo(const TimePoint& tp, double theta) {
        double dt = 0.0, dTheta = 0.0;
        if(!mThetaInfos.empty()) {
            dt = durationCastDouble(tp - std::get<0>(mThetaInfos.back()));
            dTheta = theta - std::get<1>(mThetaInfos.back());
        }
        if(mThetaInfos.size() >= mConfig.fanQueueLength){
            auto top = mThetaInfos.front();
            mThetaInfos.pop_front();
            if (mMode == TaskMode::BigRune){
                mTrackFan.state(0) -= std::get<2>(top);  //reset t0
            }
        }
            
        mThetaInfos.emplace_back(tp, theta, dt, dTheta);
        mDirSum = dTheta >= 0 ? mDirSum + 1 : mDirSum - 1;
        mDirection = mDirSum >= 0 ? 1 : -1;
    }

    void fitParameters() {
        logInfo(fmt::format("mThetaInfo size : {}", mThetaInfos.size()));
        if(mThetaInfos.size() < mConfig.fanQueueLength) {
            return;
        }

        ceres::Problem problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;

        double t0 = 0.0, theta0 = 0.0; 
        for(const auto& thetaInfo : mThetaInfos) {
            t0 += std::get<2>(thetaInfo), theta0 += std::get<3>(thetaInfo);
            problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 4>(new CurveFittingCost(t0, theta0)),
                                     nullptr, mParameters);
        }

        problem.SetParameterLowerBound(mParameters, 0, 0.780);
        problem.SetParameterUpperBound(mParameters, 0, 1.045);
        problem.SetParameterLowerBound(mParameters, 1, 1.884);
        problem.SetParameterUpperBound(mParameters, 1, 2.000);
        problem.SetParameterLowerBound(mParameters, 2, 0.0);
        problem.SetParameterUpperBound(mParameters, 2, 4.0);
        ceres::Solve(options, &problem, &summary);
        //        logInfo(summary.FullReport());
        logInfo(fmt::format("Final Cost: {:.3f} Param: {:.3f} {:.3f} {:.3f} {:.3f}", summary.final_cost, mParameters[0],
                            mParameters[1], mParameters[2], mParameters[3]));
        
    }

    std::tuple<bool, double, double> solveAngle(const Eigen::VectorXd& X) {
        int cnt = 0;
        double predictTime = 0.0;
        double w = X(1), theta = X(2);
        // logInfo(fmt::format("solve theta = {}, w = {}", theta, w));
        while(cnt < 10) {
            Eigen::VectorXd statePos = mTrackFan.state;
            if(mMode == TaskMode::SmallRune) {
                statePos(2) = theta + w * predictTime;
            } else {
                statePos(2) = clcBigRuneTheta(X(0), predictTime);
            }

            Eigen::VectorXd predictPos = getFanPosFromState(statePos);
            auto [acess2, airTime, yaw, pitch] =
                solveWithoutAirDrag({ predictPos(1), -predictPos(3), predictPos(2) }, { 0, 0, 0 });
            double requiredTime = airTime + mConfig.delay + GlobalSettings::get().latency;
            double diiffTime = requiredTime - predictTime;
            // HubLogger::watch("diffTime", diiffTime);
            // logInfo(fmt::format("{}th predictTime: {:.3f} requiredTime: {:.3f}", cnt, predictTime, requiredTime));
            if(std::fabs(diiffTime) < diffTThresh) {
                return std::make_tuple(true, yaw, pitch);
                break;
            }
            ++cnt;
            predictTime = requiredTime;
        }
        logInfo(fmt::format("Energy Predictor solve angle error occurred! iter_cnt={}", cnt));
        return { false, 0, 0 };
    }

public:
    EnergyPredictor(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, name }, mKey{ generateKey(this) },
          mTrackFan{ TimePoint(), Eigen::VectorXd::Zero(7), FanTrackingState::LOST } {
        mLostCount = mConfig.lostCnt;

        int nX = 7;  // state:t w theta xr yr zr yaw
        int nZ = 5;  // measure: theta xf yf zf yaw
        auto f = [this](const Eigen::VectorXd& X) {
            Eigen::VectorXd xNew = X;
            double a = mParameters[0], w = mParameters[1], p = mParameters[2];
            xNew(0) = X(0) + mDt;  // t += dt                                                   // t += dt
            if(mMode == TaskMode::SmallRune) {
                xNew(1) = X(1);  // w = w
            } else {
                xNew(1) = (a * std::sin(w * X(0) + p) + (2.090 - a)) * mDirection;  // w = a * sin(wt + p) + (2.090 - a);
            }
            xNew(2) += xNew(1) * mDt;  // theta += w * dt
            return xNew;
        };

        auto JF = [this, nX](const Eigen::VectorXd& X) {
            Eigen::MatrixXd F(nX, nX);
            double a = mParameters[0], w = mParameters[1], p = mParameters[2];
            if(mMode == TaskMode::SmallRune) {
                // clang-format off
                F << 1,   0, 0, 0, 0, 0, 0,
                     0,   1, 0, 0, 0, 0, 0,
                     0, mDt, 1, 0, 0, 0, 0,
                     0,   0, 0, 1, 0, 0, 0,
                     0,   0, 0, 0, 1, 0, 0,
                     0,   0, 0, 0, 0, 1, 0,
                     0,   0, 0, 0, 0, 0, 1;
                // clang-format on
                return F;
            } else {
                // clang-format off
                F << 1,                                                           0,   0, 0, 0, 0, 0,
                     (a * w * std::cos(w * X(0) + p) + (2.090 - a)) * mDirection, 0,   0, 0, 0, 0, 0,
                     0,                                                           mDt, 1, 0, 0, 0, 0,
                     0,                                                           0,   0, 1, 0, 0, 0,
                     0,                                                           0,   0, 0, 1, 0, 0,
                     0,                                                           0,   0, 0, 0, 1, 0,
                     0,                                                           0,   0, 0, 0, 0, 1;
                // clang-format on
                return F;
            }
        };

        auto h = [nZ, this](const Eigen::VectorXd& X) {
            Eigen::VectorXd Z(nZ);
            Z = getFanPosFromState(X);
            return Z;
        };
        auto JH = [nX, nZ](const Eigen::VectorXd& X) {
            Eigen::MatrixXd h(nZ, nX);
            double theta = X(2), yaw = X(6);
            // clang-format off
            h << 0, 0, 1,                                         0, 0, 0, 0,
                 0, 0, -fanLen * std::sin(theta) * std::cos(yaw), 1, 0, 0, -fanLen * std::cos(theta) * std::sin(yaw),
                 0, 0, fanLen * std::cos(theta),                  0, 1, 0, 0,
                 0, 0, -fanLen * std::sin(theta) * std::sin(yaw), 0, 0, 1, fanLen * std::cos(theta)* std::cos(yaw),
                 0, 0, 0,                                         0, 0, 0, 1;
            // clang-format on
            return h;
        };
        auto Q = [this, nX]() {
            Eigen::MatrixXd Q(nX, nX);
            // clang-format off
            Q << 0, 0,            0,              0,            0,            0,            0,             
                 0, mConfig.Qw,   0,              0,            0,            0,            0,
                 0, 0,            mConfig.Qtheta, 0,            0,            0,            0,
                 0, 0,            0,              mConfig.Qxyz, 0,            0,            0,
                 0, 0,            0,              0,            mConfig.Qxyz, 0,            0,
                 0, 0,            0,              0,            0,            mConfig.Qxyz, 0, 
                 0, 0,            0,              0,            0,            0,            mConfig.Qyaw;
            // clang-format on
            return Q;
        };
        auto R = [this, nZ](const Eigen::VectorXd& z) {
            Eigen::MatrixXd R(nZ, nZ);
            // clang-format off
            double r0 = mConfig.Rtheta, r1 = mConfig.Rxyz, r2 = mConfig.Ryaw;
            R << r0,             0,              0,              0,              0,
                 0,              r1 * abs(z[0]), 0,              0,              0,
                 0,              0,              r1 * abs(z[1]), 0,              0,
                 0,              0,              0,              r1 * abs(z[2]), 0,
                 0,              0,              0,              0,              r2;
            // clang-format on
            return R;
        };

        Eigen::MatrixXd p0(nX, nX);
        // clang-format off
        p0 << 0, 0, 0, 0, 0, 0, 0,
              0, 1, 0, 0, 0, 0, 0,
              0, 0, 1, 0, 0, 0, 0,
              0, 0, 0, 1, 0, 0, 0,
              0, 0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 0, 1;
        // clang-format on
        mFilter = ExtendedKalmanFilter{ f, h, JF, JH, Q, R, p0 };
        mTrackFan.trackSate = FanTrackingState::LOST;
    }
    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](energy_detect_available_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(energy_detect_available_atom, TypedIdentifier<EnergyFan>);

                     if(mConfig.setMode == 0) {
                         mMode = GlobalSettings::get().getTaskMode();
                     } else {
                         mMode = static_cast<TaskMode>(mConfig.setMode);
                     }

                     auto srcFan = BlackBoard::instance().get<EnergyFan>(key).value();
                     double dt = durationCastDouble(srcFan.lastUpdate - mTrackFan.lastUpdate);
                     mTrackFan.lastUpdate = srcFan.lastUpdate;
                     if(srcFan.lastUpdate - mLastHit > std::chrono::seconds(10)) {
                         reset();
                     }
                     mLastHit = srcFan.lastUpdate;

                     bool matched = false;
                     if(!srcFan.keyPoints.empty()) {
                         mTfCamera2Robot = srcFan.cameraInfo.tfRobot2Camera.invTransformObj();
                         auto [success, posRefCamera, theta, rMatCamera] = pnpSolver(srcFan.keyPoints, srcFan.cameraInfo);
                         if(!success) {
                             logError("Energy detector pnp solve failed!");
                         } else {
                             saveThetaInfo(srcFan.lastUpdate, theta);
                             glm::dvec3 posRefRobot =
                                 mTfCamera2Robot(Point<UnitType::Distance, FrameOfRef::Camera>(posRefCamera)).mVal;

                             auto rmat = combine(mTfCamera2Robot, Transform<FrameOfRef::Armor, FrameOfRef::Camera>(rMatCamera));
                             double fanYaw = normalizeAngle(atan2(rmat.raw()[2][2], rmat.raw()[2][0]) + glm::half_pi<double>());
                             fanYaw = mTrackFan.state(6) + normalizeAngle(fanYaw - mTrackFan.state(6));
                             theta = mTrackFan.state(2) + normalizeAngle(theta - mTrackFan.state(2));
                             HubLogger::watch("detectedRuneX", posRefRobot.x);
                             HubLogger::watch("detectedRuneY", posRefRobot.y);
                             HubLogger::watch("detectedRuneZ", posRefRobot.z);
                             HubLogger::watch("runeYaw", fanYaw);
                             HubLogger::watch("runeTheta", theta);

                             Eigen::VectorXd fanPos(5);
                             fanPos << theta, posRefRobot.x, posRefRobot.y, posRefRobot.z, fanYaw;
                             // std::cout << "EnergyPredicotr: mesured fanPos: theta x y z yaw" << std::endl;
                             // std::cout << fanPos << std::endl;
                             HubLogger::visualLog(
                                 fmt::format("measrued fanPos: theta x y z yaw {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}", fanPos(0),
                                             fanPos(1), fanPos(2), fanPos(3), fanPos(4)));
                             if(mTrackFan.trackSate == FanTrackingState::LOST) {
                                 init(fanPos);
                                 matched = true;
                             } else {
                                 if(mMode == TaskMode::BigRune){
                                    fitParameters();
                                 }
                                 matched = update(dt, fanPos);
                             };
                         }
                     }
                     changeTrackingState(matched);

                     if(mTrackFan.trackSate != FanTrackingState::LOST) {
                         HubLogger::watch("runeW", mTrackFan.state(1));
                         HubLogger::watch("runeTheta", mTrackFan.state(2));
                         HubLogger::watch("rLabelX", mTrackFan.state(3));
                         HubLogger::watch("rLabelY", mTrackFan.state(4));
                         HubLogger::watch("rLabelZ", mTrackFan.state(5));
                         HubLogger::watch("rLabelYaw", mTrackFan.state(6));
                         auto [success, yaw, pitch] = solveAngle(mTrackFan.state);
                         HubLogger::visualLog(fmt::format("pnp solver result: {} {:.3f} {:.3f}", success, yaw, pitch));
                         if(success) {
                             sendAllHighPriority(set_target_info_atom_v, mGroupMask, srcFan.lastUpdate.time_since_epoch().count(),
                                                 yaw, pitch, true, normalSolver);
                         }
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(EnergyPredictor);
