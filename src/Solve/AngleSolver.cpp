#include "BlackBoard.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "ExceptionProbe.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"
#include "Timer.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/fwd.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/string_cast.hpp>
#include <magic_enum.hpp>
#include <optional>
#include <vector>

#include "SuppressWarningEnd.hpp"

struct AngleSolverSettings final {
    double delay;
    bool gimbalFixed;
    double sameTimeThreshold;
    double requiredTimeWeight;
    double maxShootDeltaTheta;  // in degree
    double lVelDiscount;
    double orietationAngle;  // in degree
};

template <class Inspector>
bool inspect(Inspector& f, AngleSolverSettings& x) {
    return f.object(x).fields(f.field("delay", x.delay).fallback(0.0), f.field("gimbalFixed", x.gimbalFixed).fallback(false),
                              f.field("sameTimeThreshold", x.sameTimeThreshold).fallback(0.05),
                              f.field("requiredTimeWeight", x.requiredTimeWeight).fallback(1),
                              f.field("maxShootDeltaTheta", x.maxShootDeltaTheta).fallback(60),
                              f.field("lVelDiscount", x.lVelDiscount).fallback(1.0),
                              f.field("orietationAngle", x.orietationAngle).fallback(37));
}

struct CandidateTarget final {
    double yawAngle;
    double pitchAngle;
    double diffAngle;
    double r;
    double height;
};

class AngleSolver final : public HubHelper<caf::event_based_actor, AngleSolverSettings, set_target_info_atom> {
    Identifier mKey;
    std::deque<double> mPastAVel;
    static double absAngleDifferece(double a, double b) {
        return std::abs(normalizeAngle(b - a));
    }
    TimePoint latestReceived;

    static constexpr glm::dvec3 tf(const glm::dvec3& ori) {
        return { ori.x, -ori.z, ori.y };
    }

    static glm::dvec3 getPos(const glm::dvec3& center, double r, double theta) {
        return { center.x + r * cos(theta), center.y + r * sin(theta), center.z };
    }

public:
    AngleSolver(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, std::move(name) }, mKey{ generateKey(this) } {
        latestReceived = TimePoint::min();
    }
    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](predict_success_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(predict_success_atom, TypedIdentifier<PredictedTarget>);
                ACTOR_EXCEPTION_PROBE();

                auto data = BlackBoard::instance().get<PredictedTarget>(key);
                if(!(data.has_value())) {
                    return;
                }
                SelectedTargetInfo res;
                // check pkg order
                if(data->lastUpdate.time_since_epoch().count() > latestReceived.time_since_epoch().count()) {
                    latestReceived = data->lastUpdate;
                } else {
                    return;
                }

                res.lastUpdate = data.value().lastUpdate;

                Vector<UnitType::Distance, FrameOfRef::Robot> posRefRobot = data->center;
                Vector<UnitType::LinearVelocity, FrameOfRef::Robot> linearVel = data->linearVel;
                RobotType targetType = data->robotType;
                auto horizontalDist = std::sqrt(square(posRefRobot.mVal.z) + square(posRefRobot.mVal.x));
                HubLogger::watch("verticalDistance", posRefRobot.mVal.y);
                HubLogger::watch("horizontalDistance", horizontalDist);

                //(forward:+y,right:+x)
                glm::dvec3 tfPos = tf(posRefRobot.mVal);
                glm::dvec3 tfLinearVel = tf(linearVel.mVal);

                const auto delayTime = mConfig.delay + GlobalSettings::get().latency;
                tfPos = { tfPos.x + delayTime * tfLinearVel.x, tfPos.y + delayTime * tfLinearVel.y,
                          tfPos.z + delayTime * tfLinearVel.z };

                auto [accessible, time, yawAngle, pitchAngle] = solveWithoutAirDrag(tfPos, tfLinearVel);
                //                     logInfo(fmt::format("x:{}, y:{}, z:{}, xVel:{}, yVel:{}, zVel:{}", tfPos.x, tfPos.y,
                //                     tfPos.z, tfLinearVel.x, tfLinearVel.y, tfLinearVel.z)); logInfo(fmt::format("time:{},
                //                     yawAngle:{}, pitch:{}", time, yawAngle, pitchAngle));
                //  HubLogger::visualLog(fmt::format(
                //      "AngleSolver: target verDist: {:.3f} horizDist: {:.3f}, solved angle yaw:{}, pitch:{}, time:{}",
                //      posRefRobot.mVal.y, horizontalDist, yawAngle, pitchAngle, time));
                if(accessible) {
                    res.isFire = true;
                    res.lastUpdate = data.value().lastUpdate;
                    res.pitchAngle = pitchAngle;
                    res.yawAngle = yawAngle;
                    res.solveType = normalSolver;
                    res.targetType = targetType;
                    res.targetPos = tfPos;
                    sendAll(set_target_info_atom_v,
                            BlackBoard::instance().updateSync<SelectedTargetInfo>(Identifier{ mKey.val }, res));
                } else {
                    HubLogger::visualLog("AngleSolver: armor inaccessable (single armor)");
                }
            },
            [&](car_predict_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(predict_success_atom, TypedIdentifier<PredictedTarget>);
                ACTOR_EXCEPTION_PROBE();

                auto data = BlackBoard::instance().get<PredictedTarget>(key);
                if(!(data.has_value())) {
                    return;
                }
                SelectedTargetInfo res;
                // check pkg order
                if(data->lastUpdate.time_since_epoch().count() > latestReceived.time_since_epoch().count()) {
                    latestReceived = data->lastUpdate;
                } else {
                    logInfo("angle solver pkg order wrong! ignore wrong order");
                    return;
                }
                res.lastUpdate = data.value().lastUpdate;
                res.targetType = data.value().robotType;
                glm::dvec3 center = tf(data->center.mVal);
                double theta = -data->yaw.mVal;

                double centerYaw = normalizeAngle(atan2(center.y, center.x) - glm::half_pi<double>());
                // glm::dvec3 lVel = tf(data->linearVel.mVal) * mConfig.lVelDiscount;
                glm::dvec3 lVel = tf(data->linearVel.mVal);

                double aVel = -data->angularVel.mVal;

                HubLogger::watch("CenterYaw", centerYaw);
                HubLogger::watch("AngleVelRefRobot", aVel);

                double R[2] = { data->radius.first, data->radius.second };
                double Z[2] = { data->y.first, data->y.second };

                {
                    // glm::dvec3 pos = getPos(center, R[0], theta);
                    // HubLogger::watch("verticalDistance", pos.z);
                    // HubLogger::watch("horizontalDistance", std::sqrt(square(pos.x) + square(pos.y)));
                }

                // solve and determine possible armor
                std::optional<double> yaw, pitch;
                int armorNum = data->armorNum;
                std::vector<CandidateTarget> candTargets;
                for(int i = 0; i < armorNum; i++) {
                    double r = R[i & 1];
                    center.z = Z[i & 1];
                    double predictTime = 0;
                    for(int iterTimes = 0; iterTimes < 5; iterTimes++) {
                        glm::dvec3 predictCenter = center + lVel * predictTime;
                        double predictTheta = theta + aVel * predictTime;
                        glm::dvec3 predictPos = getPos(predictCenter, r, predictTheta);

                        auto [accessible, airTime, yawAngle, pitchAngle] = solveWithoutAirDrag(predictPos, lVel);
                        if(!accessible) {
                            // HubLogger::logInfoBoth(fmt::format("AngleSolver: {}th armor gets inaccessible", i));
                            break;
                        }
                        double requiredTime = airTime + mConfig.delay + GlobalSettings::get().latency;
                        double requiredTheta = theta + aVel * requiredTime;

                        if(requiredTime - predictTime <= mConfig.sameTimeThreshold) {
                            double deltaTheta = normalizeAngle(requiredTheta - yawAngle - glm::pi<double>());
                            if(r == 0 || std::abs(deltaTheta) <= glm::radians(mConfig.maxShootDeltaTheta)) {
                                double angleDiff =
                                    absAngleDifferece(centerYaw, normalizeAngle(yawAngle - glm::half_pi<double>()));
                                CandidateTarget candTarget;
                                candTarget.yawAngle = yawAngle;
                                candTarget.pitchAngle = pitchAngle;
                                candTarget.diffAngle = angleDiff;
                                candTarget.r = r;
                                candTarget.height = center.z;
                                candTargets.push_back(candTarget);
                                res.targetPos = predictPos;
                            } else {
                                HubLogger::visualLog(fmt::format(
                                    "AngleSolver: {}th armor deltaTheta:{:.3f} do not satisfy maxShootDelatYaw", i, deltaTheta));
                            }
                            break;
                        }
                        predictTime += mConfig.requiredTimeWeight * (requiredTime - predictTime);
                    }
                    theta += (aVel < 0 ? glm::two_pi<double>() / armorNum : -glm::two_pi<double>() / armorNum);
                }

                if(!candTargets.empty()) {
                    bool fire = false;

                    std::sort(candTargets.begin(), candTargets.end(),
                              [](const CandidateTarget& a, const CandidateTarget& b) { return a.diffAngle < b.diffAngle; });
                    yaw = candTargets[0].yawAngle;
                    pitch = candTargets[0].pitchAngle;

                    if(mConfig.gimbalFixed) {
                        double r = candTargets[0].r;
                        center.z = candTargets[0].height;
                        auto armorFaced = getPos(
                            center, r, normalizeAngle(glm::half_pi<double>() + centerYaw));  // armor_yaw - center_yaw - half_pi
                        auto [accessible, airTime, yawAngle, pitchAngle] = solveWithoutAirDrag(armorFaced, lVel);
                        if(accessible) {
                            fire = (absAngleDifferece(yaw.value(), yawAngle) < (glm::radians(mConfig.orietationAngle) + 4e-4));
                            yaw = yawAngle;
                            pitch = pitchAngle;
                        }
                    }
                    HubLogger::watch("fire", fire);
                    HubLogger::visualLog(
                        fmt::format("AngleSolver: target {}th armor yaw: {:.3f} pitch: {:.3f}", 0, yaw.value(), pitch.value()));

                    res.pitchAngle = pitch.value();
                    res.yawAngle = yaw.value();
                    res.isFire = fire;
                    res.solveType = normalSolver;
                    sendAll(set_target_info_atom_v,
                            BlackBoard::instance().updateSync<SelectedTargetInfo>(Identifier{ mKey.val }, res));
                    return;
                }
            },
        };
    }
};

HUB_REGISTER_CLASS(AngleSolver);