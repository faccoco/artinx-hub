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
static constexpr double minRadiusThreshold = 0.1;
static constexpr double maxRadiusThreshold = 0.5;
static constexpr double standardDeviationThreshold = 0.025;
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

    std::deque<std::pair<TimePoint, Vector<UnitType::Distance, FrameOfRef::Robot>>> mLastPosition;

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
        double Xcenter, Zcenter;
        if(det == 0) {
            Xcenter = 0;
            Zcenter = 0;
        } else {
            Xcenter = (Mxl * (Mzz - x) - Mzl * Mxz) / det / 2;
            Zcenter = (Mzl * (Mxx - x) - Mxl * Mxz) / det / 2;
        }
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
                auto tfGun2Robot = data->tfRobot2Gun->invTransformObj();
                HubLogger::watch("armor type", magic_enum::enum_name(data->selected->type));

                PredictedOutpost res;
                res.lastUpdate = data->lastUpdate;

                Vector<UnitType::Distance, FrameOfRef::Gun> posOfRefGun(data->selected->center.mVal);
                Vector<UnitType::Distance, FrameOfRef::Robot> posRefRobot = tfGun2Robot(posOfRefGun);

                if(mLastPosition.size() == dequeLength)
                    mLastPosition.pop_front();
                mLastPosition.push_back(std::make_pair(data->lastUpdate, posRefRobot));
                if(mLastPosition.size() < dequeLength)
                    return;

                auto [center, radius] = CircleFitByTaubin(mLastPosition);

                double standardDeviation = 0;
                for(const auto& pos : mLastPosition)
                    standardDeviation += glm::distance(center, pos.second.mVal) - radius;
                standardDeviation = std::sqrt(standardDeviation / dequeLength);
                logInfo(fmt::format("standardDeviation: {}", standardDeviation));

                // static or all points on a line
                if(radius < minRadiusThreshold || radius > maxRadiusThreshold || standardDeviation > standardDeviationThreshold) {
                    res.centerOfOutpost = mLastPosition.back().second;
                    res.angularVelocity = 0;
                    res.radius = 0;
                    res.theta = glm::radians<double>(90);
                    logInfo("predictor: static");
                    logInfo(fmt::format("center:{},{},{} radius:{}", center.x, center.y, center.z));
                } else {
                    res.centerOfOutpost = center;

                    auto baseTime = mLastPosition[0].first.time_since_epoch().count();
                    std::vector<std::pair<double, double>> time_theta;
                    time_theta.reserve(dequeLength);
                    double dTheta = 0, baseTheta = getTheta(center, tfGun2Robot.raw() * glm::dvec4{ 1, 0, 0, 0 });
                    for(const auto& pt : mLastPosition) {
                        double nowTheta = getTheta(center, pt.second.mVal) - baseTheta;
                        if(time_theta.size() > 0 && std::abs(nowTheta - time_theta.back().second) > maxJumpTheta) {
                            dTheta += (nowTheta > time_theta.back().second ? (-deltaTheta) : deltaTheta);
                        }
                        time_theta.push_back(std::make_pair(double(pt.first.time_since_epoch().count() - baseTime) /
                                                                Clock::period::den * Clock::period::num,
                                                            nowTheta + dTheta));
                        // logInfo(fmt::format("nowTheta:{}", nowTheta));
                    }
                    auto [k, m] = FitLine(time_theta);
                    res.angularVelocity = k;
                    res.theta = k * time_theta.back().first + m - dTheta;
                    logInfo(fmt::format("center:{},{},{} radius:{}\nangularVelocity:{}\ntheta:{}", res.centerOfOutpost.mVal.x,
                                        res.centerOfOutpost.mVal.y, res.centerOfOutpost.mVal.z, radius, res.angularVelocity.mVal,
                                        res.theta.mVal));
                }

                sendAll(outpost_predict_success_atom_v,
                        BlackBoard::instance().updateSync<PredictedOutpost>(Identifier{ mKey.val }, res));
            },
        };
    }
};

HUB_REGISTER_CLASS(OutpostPredictor);
