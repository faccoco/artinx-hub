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

static constexpr double minPositionDiff = 0.5;
static constexpr double minThetaDiff = 0.2;
static constexpr double fanLen = 0.675;
constexpr double longRuneArmorWidth = 0.3524;
constexpr double shortRuneArmorWidth = 0.338;
constexpr double runeArmorHeight = 0.3524;
constexpr double runeRHeight = 0.700;

struct EnergyPredictorSettings final {
    uint8_t fanQueueLength;  // 50
    std::vector<double> Q;
    std::vector<double> R;
};

template <class Inspector>
bool inspect(Inspector& f, EnergyPredictorSettings& x) {
    return f.object(x).fields(f.field("fanQueueLength", x.fanQueueLength),
                              f.field("Q", x.Q).invariant([](auto& c) { return c.size() == 6; }),
                              f.field("R", x.R).invariant([](auto& c) { return c.size() == 5; }));
}

struct CurveFittingCost {
    CurveFittingCost(double t_, double theta_) : t(t_), theta(theta_) {}

    template <typename T>
    bool operator()(const T* params, T* residual) const {
        residual[0] = T(theta) -
            (-params[0] / params[1] * ceres::cos(params[1] * T(t) + params[2]) + (2.090 - params[0]) * T(t) + params[3]);
        return true;
    }

    const double t, theta;
};

class EnergyPredictor final : public HubHelper<caf::event_based_actor, EnergyPredictorSettings, set_target_info_atom> {

    Identifier mKey;
    int mMode = 0, mDirection = 0;  // mMode energy mode(1 small, 2 big), mDirection (-1 Clockwise, 1 anti-clockwise)
    int mDirSum = 0;
    constexpr static double diffTThresh = 1e-3;
    constexpr static int mLostCountThresh = 10;
    double mParameters[4]{ 0.780, 1.884, 0.0, 0.0 };
    double mDt;
    int mLostCount = 0;
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
    }

    void init(const Eigen::VectorXd& fanPos) {
        double theta = fanPos(0), yaw = fanPos(4);
        double xr = fanPos(1) - fanLen * std::cos(theta) * std::cos(yaw), yr = fanPos(2) - fanLen * std::sin(theta),
               zr = fanPos(3) + fanLen * std::cos(theta) * std::sin(yaw);
        logInfo(fmt::format("EnergyPredictor: xr yr, zr ({:.3f}, {:.3f}, {:.3f})", xr, yr, zr));
        double initAVel = 0;
        if(2 == mMode)
            initAVel = 12.5;  // TODO
        mTrackFan.state << 0, initAVel, theta, xr, yr, zr, yaw;
        mFilter.setState(mTrackFan.state);
        logInfo("EnergyPredictor: Init EKF!");
    }

    bool update(double dt, const Eigen::VectorXd& fanPosition) {
        mDt = dt;
        auto ekfPredict = mFilter.predict();
        mTrackFan.state = ekfPredict;
        auto fanPredict = getFanPosFromState(ekfPredict);
        double positionDiff = (fanPosition.tail(4) - fanPredict.tail(4)).norm();
        if(positionDiff < minPositionDiff) {
            double thetaDiff = std::abs(fanPosition(0) - fanPredict(0));
            logInfo(fmt::format("Energy Predictor: positionDiff {:.3f} and thetaDiff {:.3f}", positionDiff, thetaDiff));
            if(thetaDiff < minThetaDiff) {
                mTrackFan.state = mFilter.update(fanPosition);
            } else {
                mTrackFan.state(0) = fanPosition(0);
                mFilter.setState(mTrackFan.state);
                mThetaInfos.clear();
            }
            return true;
        } else {
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

    double clcThetaVel(double dt) {
        double a = mParameters[0], w = mParameters[1], p = mParameters[2];
        return a * std::sin(w * dt + p) + (2.090 - a);
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
                                                            { +longRuneArmorWidth / 2, -runeArmorHeight / 2, 0.0 },
                                                            { -longRuneArmorWidth / 2, -runeArmorHeight / 2, 0.0 } };
        std::vector<cv::Point2f> imagePoints(4);
        for(int i = 0; i < 4; ++i) {
            imagePoints[i].x = keyPoints[i].x;
            imagePoints[i].y = keyPoints[i].y;
        }

        cv::Mat rvec, tvec;
        const auto pnpRes = cv::solvePnP(mFanObjectPoints, imagePoints, cameraInfo.cameraMatrix, cameraInfo.distCoefficients,
                                         rvec, tvec, false, cv::SOLVEPNP_IPPE);
        if(!pnpRes)
            return std::tuple<bool, glm::dvec3, double, glm::dmat4>{ false, 0, 0, 0 };

        glm::dvec3 fanCenter = { tvec.at<double>(0, 0), -tvec.at<double>(1, 0), -tvec.at<double>(2, 0) };
        glm::dvec3 rvecRefCam = { rvec.at<double>(0, 0), -rvec.at<double>(1, 0), -rvec.at<double>(2, 0) };

        double angle = glm::length(rvecRefCam);
        auto axis = rvecRefCam / angle;
        auto rmat = glm::mat4_cast(glm::angleAxis(-angle, axis));

        double theta = clcTheta(keyPoints[4], clcFanImgCenter(imagePoints));
        return std::make_tuple(pnpRes, fanCenter, theta, rmat);
    }

    void saveThetaInfo(const TimePoint& tp, double theta) {
        double dt = 0.0, dTheta = 0.0;
        if(mThetaInfos.size() > 0) {
            dt = durationCastDouble(tp - std::get<0>(mThetaInfos.back()));
            dTheta = theta - std::get<1>(mThetaInfos.back());
        }
        mThetaInfos.emplace_back(tp, theta, dt, dTheta);
        if(mThetaInfos.size() >= mConfig.fanQueueLength) {
            mThetaInfos.pop_front();
        }
        mDirSum = dTheta > 0 ? mDirSum + 1 : mDirSum - 1;
        mDirection = mDirSum > 0 ? 1 : -1;
    }

    void fitParameters() {
        if(mThetaInfos.size() < mConfig.fanQueueLength) {
            return;
        }

        ceres::Problem problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;

        for(auto it = mThetaInfos.begin(); it != mThetaInfos.end(); ++it) {
            auto dt = std::get<2>(*it), dTheta = std::get<3>(*it);
            problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 4>(new CurveFittingCost(dt, dTheta)),
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

    Eigen::VectorXd getFanPosFromState(const Eigen::VectorXd& X) {
        Eigen::VectorXd fanPos(5);
        double rLabelX = X(3), rLabelY = X(4), rLabeZ = X(5);
        double theta = X(2), yaw = X(6);
        fanPos(0) = theta;
        fanPos(1) = rLabelX + fanLen * std::cos(theta) * std::cos(yaw);
        fanPos(2) = rLabelY + fanLen * std::sin(theta);
        fanPos(3) = rLabeZ - fanLen * std::cos(theta) * std::sin(yaw);
        fanPos(4) = yaw;
        return fanPos;
    }

    std::pair<double, double> solveAngle(const Eigen::VectorXd& X) {
        int cnt = 0;
        double t1 = 0.0;
        double w = X(1), theta = X(2);
        while(cnt <= 5) {
            theta += w * t1 * mDirection;
            Eigen::VectorXd statePos = mTrackFan.state;
            statePos(2) = theta;
            Eigen::VectorXd predictPos = getFanPosFromState(statePos);
            std::cout << "EnergyPredictor: theta x y z yaw " << predictPos << std::endl; 
            auto [acess2, t2, yaw, pitch] = solveWithoutAirDrag({ predictPos(1), -predictPos(3), predictPos(2) }, { 0, 0, 0 });
            if(std::fabs(t2 - t1) < diffTThresh) {
                return std::make_pair(yaw, pitch);
                break;
            }
            ++cnt;
            t1 = t2;
        }
        logInfo("Energy Predictor solve angle error occurred!");
        return {};
    }

public:
    EnergyPredictor(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ generateKey(this) },
          mTrackFan{ TimePoint(), Eigen::VectorXd::Zero(7), FanTrackingState::LOST } {
        int nX = 7;  // state:t w theta xr yr zr yaw
        int nZ = 5;  // measure: theta xf yf zf yaw
        auto f = [this](const Eigen::VectorXd& X) {
            Eigen::VectorXd xNew = X;
            double a = mParameters[0], w = mParameters[1], p = mParameters[2];
            xNew(0) = X(0) + mDt;  // t += dt
            if(1 == mMode) {
                xNew(1) = X(1);  // w = w
            } else {
                xNew(1) = a * std::sin(w * X(0) + p) + (2.090 - a);  // w = a * sin(wt + p) + (2.090 - a);
            }
            xNew(2) += xNew(1) * mDt;  // theta += w * dt
            return xNew;
        };
        auto JF = [this, nX](const Eigen::VectorXd& X) {
            Eigen::MatrixXd F(nX, nX);
            double a = mParameters[0], w = mParameters[1], p = mParameters[2];
            if(1 == mMode) {
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
                F << 1,                                            0,   0, 0, 0, 0, 0,
                     a * w * std::cos(w * X(0) + p) + (2.090 - a), 0,   0, 0, 0, 0, 0,
                     0,                                            mDt, 1, 0, 0, 0, 0,
                     0,                                            0,   0, 1, 0, 0, 0,
                     0,                                            0,   0, 0, 1, 0, 0,
                     0,                                            0,   0, 0, 0, 1, 0,
                     0,                                            0,   0, 0, 0, 0, 1;
                // clang-format on
                return F;
            }
        };

        auto h = [nZ](const Eigen::VectorXd& X) {
            Eigen::VectorXd Z(nZ);
            double theta = X(2), yaw = X(6);
            Z(0) = theta;  // theta
            Z(1) = X(3) + fanLen * std::cos(theta) * std::cos(yaw);
            Z(2) = X(4) + fanLen * std::sin(theta);
            Z(3) = X(5) - fanLen * std::cos(theta) * std::sin(yaw);
            Z(4) = yaw;
            return Z;
        };
        auto JH = [nX, nZ](const Eigen::VectorXd& X) {
            Eigen::MatrixXd h(nZ, nX);
            double theta = X(2), yaw = X(6);
            // clang-format off
            h << 0, 0, 1,                                              0, 0, 0, 0,
                 0, 0, -fanLen * std::sin(theta) * std::cos(yaw), 1, 0, 0, -fanLen * std::cos(theta) * std::sin(yaw),
                 0, 0, fanLen * std::cos(theta),                    0, 1, 0, 0,
                 0, 0, fanLen * std::sin(theta) * std::sin(yaw),  0, 0, 1, -fanLen * std::cos(theta)* std::cos(yaw),
                 0, 0, 0,                                              0, 0, 0, 1;
            // clang-format on
            return h;
        };
        auto Q = [this, nX]() {
            Eigen::MatrixXd Q(nX, nX);
            // clang-format off
            Q << 0, 0,            0,            0,            0,            0,            0,             
                 0, mConfig.Q[0], 0,            0,            0,            0,            0,
                 0, 0,            mConfig.Q[1], 0,            0,            0,            0,
                 0, 0,            0,            mConfig.Q[2], 0,            0,            0,
                 0, 0,            0,            0,            mConfig.Q[3], 0,            0,
                 0, 0,            0,            0,            0,            mConfig.Q[4], 0, 
                 0, 0,            0,            0,            0,            0,            mConfig.Q[5];
            // clang-format on
            return Q;
        };
        auto R = [this, nZ](const Eigen::VectorXd&) {
            Eigen::MatrixXd R(nZ, nZ);
            // clang-format off
            R << mConfig.R[0], 0,            0,            0,            0,
                 0,            mConfig.R[1], 0,            0,            0,
                 0,            0,            mConfig.R[2], 0,            0,
                 0,            0,            0,            mConfig.R[3], 0,
                 0,            0,            0,            0,            mConfig.R[4];
            // clang-format on
            return R;
        };

        Eigen::MatrixXd p0(nX, nX);
        // clang-format off
        p0 << 1, 0, 0, 0, 0, 0, 0,
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
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [&](energy_detect_available_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(energy_detect_available_atom, TypedIdentifier<EnergyFan>);
                mMode = 1;
                auto srcFan = BlackBoard::instance().get<EnergyFan>(key).value();

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
                        double fanYaw = normalizeAngle(-atan2(rmat.raw()[2][0], rmat.raw()[2][2]) - glm::half_pi<double>());
                        fanYaw = mTrackFan.state(6) + normalizeAngle(fanYaw - mTrackFan.state(6));
                        HubLogger::watch("rune_x", posRefRobot.x);
                        HubLogger::watch("rune_y", posRefRobot.y);
                        HubLogger::watch("rune_z", posRefRobot.z);
                        HubLogger::watch("yaw", fanYaw);
                        HubLogger::watch("theta", theta);

                        Eigen::VectorXd fanPos(5);
                        fanPos << theta, posRefRobot.x, posRefRobot.y, posRefRobot.z, fanYaw;
                        std::cout << "EnergyPredictor: measured postion: theta x y z yaw " << fanPos << std::endl;
                        if(mTrackFan.trackSate == FanTrackingState::LOST) {
                            init(fanPos);
                            matched = true;
                        } else {
                            double dt = durationCastDouble(srcFan.lastUpdate - mTrackFan.lastUpdate);
                            if(2 == mMode)
                                fitParameters();
                            matched = update(dt, fanPos);
                        };
                    }
                }
                changeTrackingState(matched);

                if(mTrackFan.trackSate != FanTrackingState::LOST) {
                    auto [yaw, pitch] = solveAngle(mTrackFan.state);
                    sendAllHighPriority(set_target_info_atom_v, mGroupMask, srcFan.lastUpdate.time_since_epoch().count(), yaw,
                                        pitch, true, normalSolver);
                }
            },
        };
    }
};

HUB_REGISTER_CLASS(EnergyPredictor);
