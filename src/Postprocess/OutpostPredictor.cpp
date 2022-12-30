#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "ExceptionProbe.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include <Eigen/Core>
#include <caf/event_based_actor.hpp>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"

static constexpr int dequeLength = 50;
static constexpr double minRadius = 0.1;
static constexpr double staticPosThreshold = 0.01;
static constexpr Duration maxWaitingTime = 50ms;
static constexpr double maxJumpTheta = glm::radians<double>(10);
static constexpr double deltaTheta = glm::radians<double>(120);

struct ArmorPredictorSettings final {
    bool enablePredictor;
};

template <class Inspector>
bool inspect(Inspector& f, ArmorPredictorSettings& x) {
    return f.object(x).fields(f.field("enablePredictor", x.enablePredictor));
}

class OutpostPredictor final : public HubHelper<caf::event_based_actor, ArmorPredictorSettings, outpost_predict_success_atom> {
    Identifier mKey;
    GroupMask mGroupMask;

    // 用预测的
    std::deque<std::pair<TimePoint, Vector<UnitType::Distance, FrameOfRef::Robot>>> mLastPosition;

    // 不用预测的
    std::queue<std::pair<TimePoint, Vector<UnitType::Distance, FrameOfRef::Robot>>> mLastTwoPosition;
    std::optional<Scalar<UnitType::Angle>> mLastTheta;
    int stopTimes = 0;

    // a,b,c mustn't be on the same line or on the same point
    glm::dvec3 circleCenter(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c) {
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

    // pair[center,radius]
    std::pair<glm::dvec3, double>
    CircleFitByTaubin(const std::deque<std::pair<TimePoint, Vector<UnitType::Distance, FrameOfRef::Robot>>>& pts) {
        static constexpr int maxIterTimes = 99;

        int n = pts.size();
        double x, y, z, meanX, meanY, meanZ, Mxx, Mzz, Mxz, Mxl, Mzl, Mll;
        meanX = meanY = meanZ = Mxx = Mzz = Mxz = Mxl = Mzl = Mll = 0;

        for(auto& pt : pts) {
            meanX += pt.second.mVal.x;
            meanY += pt.second.mVal.y;
            meanZ += pt.second.mVal.z;
        }
        meanX /= n;
        meanY /= n;
        meanZ /= n;

        for(auto& pt : pts) {
            double xi = pt.second.mVal.x - meanX;
            double zi = pt.second.mVal.z - meanZ;
            double li = xi * xi + zi * zi;
            Mxx += xi * xi;
            Mzz += zi * zi;
            Mxz += xi * zi;
            Mxl += xi * li;
            Mzl += zi * li;
            Mll += li * li;
        }
        Mxx /= n;
        Mzz /= n;
        Mxz /= n;
        Mxl /= n;
        Mzl /= n;
        Mll /= n;

        double Ml = Mxx + Mzz;
        double Cov_xz = Mxx * Mzz - Mxz * Mxz;
        double Var_l = Mll - Ml * Ml;
        double A3 = 4 * Ml;
        double A2 = -3 * Ml * Ml - Mll;
        double A1 = Var_l * Ml + 4 * Cov_xz * Ml - Mxl * Mxl - Mzl * Mzl;
        double A0 = Mxl * (Mxl * Mzz - Mzl * Mxz) + Mzl * (Mzl * Mxx - Mxl * Mxz) - Var_l * Cov_xz;
        double A22 = 2 * A2;
        double A33 = 3 * A3;

        int i;
        for(x = 0, z = A0, i = 0; i < maxIterTimes; i++) {
            double xnew = x - z / (A1 + x * (A22 + A33 * x));
            if((xnew == x) || (!std::isfinite(xnew)))
                break;
            double znew = A0 + xnew * (A1 + xnew * (A2 + xnew * A3));
            if(std::abs(znew) >= std::abs(z))
                break;
            x = xnew;
            z = znew;
        }

        double det = x * x - x * Ml + Cov_xz;
        double Xcenter = (Mxl * (Mzz - x) - Mzl * Mxz) / det / 2;
        double Zcenter = (Mzl * (Mxx - x) - Mxl * Mxz) / det / 2;
        x = Xcenter + meanX;
        y = meanY;
        z = Zcenter + meanZ;

        return std::make_pair(glm::dvec3{ x, y, z }, std::sqrt(Xcenter * Xcenter + Zcenter * Zcenter + Ml));
    }

    // pair[k,m]
    std::pair<double, double> FitLine(std::vector<std::pair<double, double>> pts) {
        int n = pts.size();
        double meanX = 0, meanY = 0, sigma1 = 0, sigma2 = 0, k, m;
        for(const auto& pt : pts) {
            meanX += pt.first;
            meanY += pt.second;
            sigma1 += pt.first * pt.second;
            sigma2 += pt.first * pt.first;
        }
        meanX /= n;
        meanY /= n;
        k = (sigma1 - n * meanX * meanY) / (sigma2 - n * meanX * meanX);
        m = meanY - k * meanX;
        return std::make_pair(k, m);
    }

    double getTheta(const glm::dvec3& center, const glm::dvec3& point) {
        double x = point.x - center.x;
        double z = point.z - center.z;
        double l = glm::sqrt(x * x + z * z);
        return glm::acos(x / l);
    }

public:
    OutpostPredictor(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](set_outpost_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(set_outpost_atom, TypedIdentifier<SelectedTarget>);
                ACTOR_EXCEPTION_PROBE();

                auto data = BlackBoard::instance().get<SelectedTarget>(key);
                if(!(data->selected.has_value() && data->tfRobot2Gun.has_value()))
                    return;
                HubLogger::watch("armor type", magic_enum::enum_name(data->selected->type));

                PredictedOutpost res;
                res.lastUpdate = data->lastUpdate;

                Vector<UnitType::Distance, FrameOfRef::Gun> posOfRefGun(data->selected->center.mVal);
                Vector<UnitType::Distance, FrameOfRef::Robot> posRefRobot = data->tfRobot2Gun->invTransform(posOfRefGun);

                if(mConfig
                       .enablePredictor) {  // 如果使用预测功能的话，使用Taubin法获取圆心位置，使用最小二乘法线性拟合获取角速度和当前位置
                    if(mLastPosition.size() == dequeLength)
                        mLastPosition.pop_front();
                    mLastPosition.push_back(std::make_pair(data->lastUpdate, posRefRobot));
                    if(mLastPosition.size() < dequeLength)
                        return;

                    auto [center, radius] = CircleFitByTaubin(mLastPosition);
                    res.centerOfOutpost = center;

                    auto baseTime = mLastPosition[0].first.time_since_epoch().count();
                    std::vector<std::pair<double, double>> time_theta;
                    time_theta.reserve(dequeLength);
                    double dTheta = 0;
                    for(const auto& pt : mLastPosition) {
                        double nowTheta = getTheta(center, pt.second.mVal);
                        if(time_theta.size() > 0 && std::abs(nowTheta - time_theta.back().second) > maxJumpTheta) {
                            nowTheta += (dTheta += (nowTheta > time_theta.back().second ? (-deltaTheta) : deltaTheta));
                        }
                        time_theta.push_back(std::make_pair(double(pt.first.time_since_epoch().count() - baseTime) /
                                                                Clock::period::den * Clock::period::num,
                                                            nowTheta));
                    }
                    auto [k, m] = FitLine(time_theta);
                    res.angularVelocity = k;
                    res.theta = k * time_theta.back().first + m - dTheta;
                    // logInfo(fmt::format("center:{},{},{}\nangularVelocity:{}\ntheta:{}", res.centerOfOutpost.mVal.x,
                    //                     res.centerOfOutpost.mVal.y, res.centerOfOutpost.mVal.z, res.angularVelocity.mVal,
                    //                     res.theta.mVal));
                } else {  // 如果不使用预测功能的话，不修正位置
                    static std::optional<int> direction;
                    if(stopTimes > 5) {
                        logInfo("outpost stop");
                        res.angularVelocity = 0;
                        res.theta = glm::radians(90.0);
                        res.centerOfOutpost = posRefRobot;
                        res.centerOfOutpost.mVal.z -= radiusOfOutpost;
                    } else {
                        if(mLastTwoPosition.size() == 0) {
                            mLastTwoPosition.push(std::make_pair(data->lastUpdate, posRefRobot));
                            return;
                        }
                        if(posRefRobot == mLastTwoPosition.back().second || posRefRobot == mLastTwoPosition.front().second) {
                            stopTimes += 1;
                            return;
                        }
                        if(mLastTwoPosition.size() == 1) {
                            mLastTwoPosition.push(std::make_pair(data->lastUpdate, posRefRobot));
                            return;
                        }
                        if(data->lastUpdate - mLastTwoPosition.front().first > maxWaitingTime) {
                            if(glm::distance(posRefRobot.mVal, mLastTwoPosition.front().second.mVal) > staticPosThreshold) {
                                logInfo("outpost remove stop");
                                stopTimes = 0;
                            } else {
                                stopTimes += 1;
                            }
                            mLastTwoPosition.pop();
                            mLastTwoPosition.push(std::make_pair(data->lastUpdate, posRefRobot));
                            mLastTheta = std::nullopt;
                            return;
                        }
                        res.centerOfOutpost = circleCenter(mLastTwoPosition.front().second.mVal,
                                                           mLastTwoPosition.back().second.mVal, posRefRobot.mVal);
                        res.theta = getTheta(res.centerOfOutpost.mVal, posRefRobot.mVal);
                        if(!mLastTheta.has_value())
                            mLastTheta = getTheta(res.centerOfOutpost.mVal, mLastTwoPosition.back().second.mVal);
                        if(!direction.has_value()) {
                            if(std::fabs((mLastTheta.value() - res.theta).mVal) > maxJumpTheta)
                                direction = (mLastTheta.value() > res.theta ? 1 : -1);
                            else
                                direction = (res.theta > mLastTheta.value() ? 1 : -1);
                        }
                        if(std::fabs((mLastTheta.value() - res.theta).mVal) > maxJumpTheta)
                            mLastTheta.value().mVal -= direction.value() * glm::radians<double>(120);
                        res.angularVelocity = (res.theta - mLastTheta.value()) /
                            Scalar<UnitType::Time>{ static_cast<double>(
                                                        (data->lastUpdate - mLastTwoPosition.back().first).count()) /
                                                    Clock::period::den * Clock::period::num };
                        mLastTheta = res.theta;
                        mLastTwoPosition.pop();
                        mLastTwoPosition.push(std::make_pair(data->lastUpdate, posRefRobot));
                    }
                    // logInfo(fmt::format("center:{},{},{}\nangularVelocity:{}\ntheta:{}", res.centerOfOutpost.mVal.x,
                    //                     res.centerOfOutpost.mVal.y, res.centerOfOutpost.mVal.z, res.angularVelocity.mVal,
                    //                     res.theta.mVal));
                }

                sendAll(outpost_predict_success_atom_v,
                        BlackBoard::instance().updateSync<PredictedOutpost>(Identifier{ mKey.val }, res));
            },
        };
    }
};

HUB_REGISTER_CLASS(OutpostPredictor);
