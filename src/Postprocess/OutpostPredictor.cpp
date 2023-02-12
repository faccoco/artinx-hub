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
static constexpr double minRadiusThreshold = 0.1;
static constexpr double maxRadiusThreshold = 0.5;
static constexpr double standardDeviationThreshold = 0.025;
static constexpr double maxJumpTheta = glm::radians<double>(10);
static constexpr double deltaTheta = glm::radians<double>(120);

class OutpostPredictor final : public HubHelper<caf::event_based_actor, void, outpost_predict_success_atom> {
    Identifier mKey;

    std::deque<std::pair<TimePoint, Vector<UnitType::Distance, FrameOfRef::Robot>>> mLastPosition;

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

                std::vector<glm::dvec3> lastPosition;
                lastPosition.reserve(dequeLength);
                for(int i = 0; i < dequeLength; i++)
                    lastPosition.push_back(mLastPosition[i].second.mVal);
                auto [center, radius] = CircleFitByTaubin(lastPosition);

                double standardDeviation = 0;
                for(const auto& pos : lastPosition)
                    standardDeviation += square(glm::distance(center, pos) - radius);
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
