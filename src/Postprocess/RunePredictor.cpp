#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "EnergyDetect.hpp"
#include "ExceptionProbe.hpp"
#include "ExtendKalmanFilter.hpp"
#include "Hub.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <ceres/ceres.h>
#include <fmt/format.h>
#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include "SuppressWarningEnd.hpp"

static constexpr double maxDtLastRecv = 0.1;

struct EnergyPredictorSettings final {
    unsigned int fanQueueLength;  // 50
    double maxResidual;
    std::vector<double> P;
    std::vector<double> Q;
    std::vector<double> R;
};

template <class Inspector>
bool inspect(Inspector& f, EnergyPredictorSettings& x) {
    return f.object(x).fields(f.field("fanQueueLength", x.fanQueueLength), f.field("maxResidual", x.maxResidual),
                              f.field("P", x.P).invariant([](auto& c) { return c.size() == 3; }),
                              f.field("Q", x.Q).invariant([](auto& c) { return c.size() == 3; }),
                              f.field("R", x.R).invariant([](auto& c) { return c.size() == 3; }));
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
    bool isInitParam = false, isCorrectPos = false;
    int mMode = 0, mDirection = 0;  // mMode energy mode(1 small, 2 big), mDirection (-1 Clockwise, 1 anti-clockwise)
    constexpr static double diffTThresh = 1e-3;
    double mParameters[4]{ 0.780, 1.884, 0.0, 0.0 };
    glm::dvec3 mRLabel;
    std::optional<TimePoint> start;
    std::deque<EnergyFan> mFans;
    std::deque<std::pair<double, double>> mThetaInfos;  //(time, theta)
    ExtendKalmanFilter mFilter;

    static std::complex<double> sqrtN(const std::complex<double>& x, double n) {
        if(auto r = std::hypot(x.real(), x.imag()); r > 0.0) {
            auto a = std::atan2(x.imag(), x.real());
            n = 1.0 / n;
            r = std::pow(r, n);
            a *= n;
            return std::polar(r, a);
        }
        return {};
    }

    static double ferrari(std::complex<double> a, std::complex<double> b, std::complex<double> c, std::complex<double> d,
                          std::complex<double> e) {
        std::complex<double> x[4];
        a = 1.0 / a;
        b *= a;
        c *= a;
        d *= a;
        e *= a;
        const auto p = (c * c + 12.0 * e - 3.0 * b * d) / 9.0;
        const auto q = (27.0 * d * d + 2.0 * c * c * c + 27.0 * b * b * e - 72.0 * c * e - 9.0 * b * c * d) / 54.0;
        const auto D = sqrtN(q * q - p * p * p, 2.0);
        std::complex<double> u = q + D;
        std::complex<double> v = q - D;
        if(v.real() * v.real() + v.imag() * v.imag() > u.real() * u.real() + u.imag() * u.imag()) {
            u = sqrtN(v, 3.0);
        } else {
            u = sqrtN(u, 3.0);
        }
        std::complex<double> y;
        if(u.real() * u.real() + u.imag() * u.imag() > 0.0) {
            v = p / u;
            const std::complex<double> o1(-0.5, +0.86602540378443864676372317075294);
            const std::complex<double> o2(-0.5, -0.86602540378443864676372317075294);
            std::complex<double>& yMax = x[0];
            double m2Max = 0.0;
            // int iMax = -1;
            for(int i = 0; i < 3; ++i) {
                y = u + v + c / 3.0;
                u *= o1;
                v *= o2;
                a = b * b + 4.0 * (y - c);
                if(const auto m2 = a.real() * a.real() + a.imag() * a.imag(); 0 == i || m2Max < m2) {
                    m2Max = m2;
                    yMax = y;
                    // iMax = i;
                }
            }
            y = yMax;
        } else {
            y = c / 3.0;
        }
        if(const auto m = sqrtN(b * b + 4.0 * (y - c), 2.0); m.real() * m.real() + m.imag() * m.imag() >= DBL_MIN) {
            const std::complex<double> n = (b * y - 2.0 * d) / m;

            a = sqrtN((b + m) * (b + m) - 8.0 * (y + n), 2.0);
            x[0] = (-(b + m) + a) / 4.0;
            x[1] = (-(b + m) - a) / 4.0;
            a = sqrtN((b - m) * (b - m) - 8.0 * (y - n), 2.0);
            x[2] = (-(b - m) + a) / 4.0;
            x[3] = (-(b - m) - a) / 4.0;
        } else {
            a = sqrtN(b * b - 8.0 * y, 2.0);
            x[0] = x[1] = (-b + a) / 4.0;
            x[2] = x[3] = (-b - a) / 4.0;
        }
        double ans = 1000;
        for(auto& i : x) {
            if(i.real() > 0 && std::fabs(i.imag()) < 1e7 && i.real() < ans)
                ans = i.real();
        }
        return ans;
    }

    void reset() {
        mFans.clear();
        mThetaInfos.clear();
        mFilter.mIsInitFilter = false;
        start.reset();
    }

    double clcThetaVel(double dt) {
        double a = mParameters[0], w = mParameters[1], p = mParameters[2];
        return a * std::sin(w * dt + p) + (2.090 - a);
    }

    void saveEnergyFan(const EnergyFan& detectedFan) {
        if(mFans.size() < mConfig.fanQueueLength) {
            mFans.push_back(detectedFan);
        } else {
            mFans.push_back(detectedFan);
            mFans.pop_front();
        }
        if(!start.has_value())
            start = mFans.begin()->lastUpdate;
    }

    void setFilterMat() {
        Eigen::MatrixXd P(4, 4);
        P << 1, 0, 0, 0,            //
            0, mConfig.P[0], 0, 0,  //
            0, 0, mConfig.P[1], 0,  //
            0, 0, 0, mConfig.P[2];
        Eigen::MatrixXd Q(4, 4);
        Q << 1, 0, 0, 0,            //
            0, mConfig.Q[0], 0, 0,  //
            0, 0, mConfig.Q[1], 0,  //
            0, 0, 0, mConfig.Q[2];
        Eigen::MatrixXd H(4, 4);
        H.setIdentity();
        Eigen::MatrixXd R(4, 4);
        R << 1, 0, 0, 0,            //
            0, mConfig.R[0], 0, 0,  //
            0, 0, mConfig.R[1], 0,  //
            0, 0, 0, mConfig.R[2];
        mFilter.setFilter(P, Q, H, R);
    }

    void setFilterFunc() {
        double a = mParameters[0], w = mParameters[1], p = mParameters[2], x = mRLabel.x, y = mRLabel.y;

        if(1 == mMode) {
            mFilter.setPredictFunc([=](const Eigen::VectorXd& X, double dt) {
                Eigen::VectorXd res(4);
                res(0) = X(0) + dt;
                res(1) = X(1);
                res(2) = x + (X(2) - x) * std::cos(X(1) * dt) - (X(3) - y) * std::sin(X(1) * dt);
                res(3) = y + (X(2) - x) * std::sin(X(1) * dt) + (X(3) - y) * std::cos(X(1) * dt);
                return res;
            });

            mFilter.setStateTFFunc([=](const Eigen::VectorXd& X, double dt) {
                Eigen::MatrixXd F(4, 4);
                F << 1, 0, 0, 0,                                                                          //
                    0, 0, 0, 0,                                                                           //
                    0, (X(2) - x) * std::sin(X(1) * dt) * dt, std::cos(X(1) * dt), -std::sin(X(1) * dt),  //
                    0, (X(2) - x) * std::cos(X(1) * dt) * dt, std::sin(X(1) * dt), std::cos(X(1) * dt);
                return F;
            });
        } else {
            mFilter.setPredictFunc([=](const Eigen::VectorXd& X, double dt) {
                Eigen::VectorXd res(4);
                res(0) = X(0) + dt;
                res(1) = a * std::sin(w * X(0) + p) + (2.090 - a);
                res(2) = x + (X(2) - x) * std::cos(X(1) * dt) - (X(3) - y) * std::sin(X(1) * dt);
                res(3) = y + (X(2) - x) * std::sin(X(1) * dt) + (X(3) - y) * std::cos(X(1) * dt);
                return res;
            });

            mFilter.setStateTFFunc([=](const Eigen::VectorXd& X, double dt) {
                Eigen::MatrixXd F(4, 4);
                F << 1, 0, 0, 0,                                                                          //
                    w * a * std::cos(w * X(0) + p), 0, 0, 0,                                              //
                    0, (X(2) - x) * std::sin(X(1) * dt) * dt, std::cos(X(1) * dt), -std::sin(X(1) * dt),  //
                    0, (X(2) - x) * std::cos(X(1) * dt) * dt, std::sin(X(1) * dt), std::cos(X(1) * dt);
                return F;
            });
        }
    }

    void correctPosition() {
        if(mFans.size() < mConfig.fanQueueLength) {
            return;
        } else {
            if(!isCorrectPos) {
                double rLabelX = 0.0, rLabelY = 0.0, rLabelZ = 0.0;
                for(auto it = mFans.begin(); it != mFans.end(); ++it) {
                    auto rLabel = it->rLabelCenter.mVal;
                    rLabelX += rLabel.x;
                    rLabelY += rLabel.y;
                    rLabelZ += rLabel.z;
                    /*            logInfo(fmt::format("rLabel: {}, {}, {}", rLabel.x, rLabel.y, rLabel.z));
                                logInfo(fmt::format("armorCenter: {} {} {}", it->armorCenter.mVal.x, it->armorCenter.mVal.y,
                                it->armorCenter.mVal.z));*/
                }
                mRLabel = glm::dvec3{ rLabelX / mConfig.fanQueueLength, rLabelY / mConfig.fanQueueLength,
                                      rLabelZ / mConfig.fanQueueLength };
                isCorrectPos = true;
                clcThetaAndDir();
                // TODO correct Armor Position
            } else {
                // TODO update Position
            }
        }
    }

    void clcThetaAndDir() {
        mThetaInfos.emplace_back(0.0, 0.0);
        const auto head = mFans.begin();
        auto lastArmor = head->armorCenter.mVal - mRLabel;
        auto lastAngelSum = 0.0;

        for(auto it = mFans.begin() + 1; it != mFans.end(); ++it) {
            double dt = clcDiffTime(it->lastUpdate, head->lastUpdate);

            glm::dvec3 armor = it->armorCenter.mVal - mRLabel;
            double rotateAngle = glm::orientedAngle(glm::normalize(lastArmor), glm::normalize(armor), glm::dvec3(0.0, 0.0, 1.0));

            if(rotateAngle > 0) {
                mDirection++;
            } else {
                mDirection--;
            }
            rotateAngle += lastAngelSum;
            mThetaInfos.emplace_back(dt, rotateAngle);

            lastArmor = armor;
            lastAngelSum = rotateAngle;
            //            logInfo(fmt::format("Theta {}", glm::degrees(rotateAngle)));
        }
        mDirection = mDirection > 0 ? 1 : -1;
    }

    void fitParameters() {
        if(2 != mMode || isInitParam)
            return;

        ceres::Problem problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;

        for(auto it = mThetaInfos.begin(); it != mThetaInfos.end(); ++it) {
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 4>(new CurveFittingCost(it->first, it->second)), nullptr,
                mParameters);
        }

        problem.SetParameterLowerBound(mParameters, 0, 0.780);
        problem.SetParameterUpperBound(mParameters, 0, 1.045);
        problem.SetParameterLowerBound(mParameters, 1, 1.884);
        problem.SetParameterUpperBound(mParameters, 1, 2.000);
        problem.SetParameterLowerBound(mParameters, 2, 0.0);
        problem.SetParameterUpperBound(mParameters, 2, 4.0);
        ceres::Solve(options, &problem, &summary);
        isInitParam = true;
        //        logInfo(summary.FullReport());
        logInfo(fmt::format("Final Cost: {:.3f} Param: {:.3f} {:.3f} {:.3f} {:.3f}", summary.final_cost, mParameters[0],
                            mParameters[1], mParameters[2], mParameters[3]));
    }

    void solveAngle(const glm::dvec3& target, double dt, double& yaw, double& pitch) {
        constexpr auto square = [=](const double x) { return x * x; };
        const auto& globalSettings = GlobalSettings::get();
        const double g = globalSettings.gForce, bulletSpeed = globalSettings.bulletSpeed;

        glm::dvec3 tfTargetPos = { target.x, -target.z, target.y }, curPos = tfTargetPos, predictPos = curPos;
        int cnt = 0;
        double t{};

        while(cnt < 5) {
            auto t1 = ferrari(1, 0, -(4 * g * curPos.z + 4 * square(bulletSpeed)) / square(g), 0.0,
                              (4 * square(curPos.x) + 4 * square(curPos.y) + 4 * square(curPos.z)) / square(g));
            double dTheta = 0.0;
            if(1 == mMode) {
                dTheta = energySpeed * t1 * mDirection;
            } else {
                dTheta = clcThetaVel(dt) * t1 * mDirection;
            }

            predictPos.x =
                mRLabel.x + (tfTargetPos.x - mRLabel.x) * std::cos(dTheta) - (tfTargetPos.z - mRLabel.y) * std::sin(dTheta);
            predictPos.z =
                mRLabel.y + (tfTargetPos.x - mRLabel.x) * std::sin(dTheta) + (tfTargetPos.z - mRLabel.y) * std::cos(dTheta);
            auto t2 = ferrari(1, 0, -(4 * g * predictPos.z + 4 * square(bulletSpeed)) / square(g), 0.0,
                              (4 * square(predictPos.x) + 4 * square(predictPos.y) + 4 * square(predictPos.z)) / square(g));
            t = t2;
            if(std::fabs(t2 - t1) < diffTThresh) {
                break;
            } else {
                curPos = predictPos;
            }
            ++cnt;
        }
        logInfo(fmt::format("Predict hit pos Ref Robot:{:.3f}, {:.3f}, {:.3f}", predictPos.x, predictPos.z, -predictPos.y));
        double angle = std::atan2(predictPos.y, predictPos.x);
        yaw = angle - glm::half_pi<double>();
        double verticalSpeed = (predictPos.z - 0.5 * g * t * t) / t;
        pitch = std::asin(verticalSpeed / bulletSpeed);
    }

public:
    EnergyPredictor(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [&](energy_detector_control_atom, uint8_t mode, double dt) {
                ACTOR_PROTOCOL_CHECK(energy_detector_control_atom, uint8_t, double);
                mMode = mode;
                if(dt > maxDtLastRecv) {
                    reset();
                }
            },
            [&](energy_detect_available_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(energy_detect_available_atom, TypedIdentifier<EnergyFan>);
                mMode = 2;
                auto srcFan = BlackBoard::instance().get<EnergyFan>(key).value();
                double yaw = 0.0, pitch = 0.0;
                saveEnergyFan(srcFan);
                correctPosition();
                if(!isCorrectPos) {
                    return;
                }
                if(2 == mMode) {
                    fitParameters();
                }

                if(!mFilter.mIsInitFilter) {
                    mFilter.setResidual(mConfig.maxResidual);
                    setFilterMat();
                    setFilterFunc();
                    mFilter.mIsInitFilter = true;
                }

                auto target = srcFan.armorCenter.mVal;
                double dt = clcDiffTime(srcFan.frame.lastUpdate, start.value());
                Eigen::VectorXd x(4);
                x << dt, clcThetaVel(dt), target.x, target.y;
                mFilter.runFilter(x, srcFan.frame.lastUpdate);
                auto fx = mFilter.getX();
                solveAngle({ fx(2), fx(3), target.z }, dt, yaw, pitch);

                logInfo(fmt::format("Armor Center Ref Robot:{:.3f} {:.3f} {:.3f}", target.x, target.y, target.z));
                logInfo(fmt::format("Clc yaw:{:.3f}, pitch:{:.3f}", yaw, pitch));
                sendAllHighPriority(set_target_info_atom_v, mGroupMask, srcFan.frame.lastUpdate.time_since_epoch().count(), yaw, pitch,
                                    true);
            },
        };
    }
};

HUB_REGISTER_CLASS(EnergyPredictor);
