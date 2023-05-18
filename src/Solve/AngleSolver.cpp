#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "ExceptionProbe.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/gtx/string_cast.hpp>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"

struct AngleSolverSettings final {
    double delay;
    double sameTimeThreshold;
    double requiredTimeWeight;
    double maxShootDeltaTheta;  // in degree
};

template <class Inspector>
bool inspect(Inspector& f, AngleSolverSettings& x) {
    return f.object(x).fields(f.field("delay", x.delay).fallback(0.0),
                              f.field("sameTimeThreshold", x.sameTimeThreshold).fallback(0.05),
                              f.field("requiredTimeWeight", x.requiredTimeWeight).fallback(1),
                              f.field("maxShootDeltaTheta", x.maxShootDeltaTheta).fallback(60));
}

class AngleSolver final : public HubHelper<caf::event_based_actor, AngleSolverSettings, set_target_info_atom> {

    static constexpr glm::dvec3 tf(const glm::dvec3& ori) {
        return { ori.x, -ori.z, ori.y };
    }

    static glm::dvec3 getPos(const glm::dvec3& center, double r, double theta) {
        return { center.x + r * cos(theta), center.y + r * sin(theta), center.z };
    }

public:
    AngleSolver(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [this](predict_success_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(predict_success_atom, TypedIdentifier<PredictedTarget>);
                     ACTOR_EXCEPTION_PROBE();

                     auto data = BlackBoard::instance().get<PredictedTarget>(key);
                     if(!(data.has_value()))
                         return;

                     Vector<UnitType::Distance, FrameOfRef::Robot> posRefRobot = data->center;
                     Vector<UnitType::LinearVelocity, FrameOfRef::Robot> linearVel = data->linearVel;
                     auto horizontalDist = std::sqrt(square(posRefRobot.mVal.z) + square(posRefRobot.mVal.x));
                     HubLogger::watch("verticalDistance", posRefRobot.mVal.y);
                     HubLogger::watch("horizontalDistance", horizontalDist);

                     //(forward:+y,right:+x)
                     glm::dvec3 tfPos = tf(posRefRobot.mVal);
                     glm::dvec3 tfLinearVel = tf(linearVel.mVal);

                     const auto delayTime = mConfig.delay + GlobalSettings::get().latency;
                     tfPos = { tfPos.x + delayTime * tfLinearVel.x, tfPos.y + delayTime * tfLinearVel.y,
                               tfPos.z + delayTime * tfLinearVel.z };

                     auto [time, yawAngle, pitchAngle] = solveWithoutAirDrag(tfPos, tfLinearVel);
                     //                     logInfo(fmt::format("x:{}, y:{}, z:{}, xVel:{}, yVel:{}, zVel:{}", tfPos.x, tfPos.y,
                     //                     tfPos.z, tfLinearVel.x, tfLinearVel.y, tfLinearVel.z)); logInfo(fmt::format("time:{},
                     //                     yawAngle:{}, pitch:{}", time, yawAngle, pitchAngle));
                     HubLogger::VisualLog(fmt::format(
                         "AngleSolver: target verDist: {:.3f} horizDist: {:.3f}, solved angle yaw:{}, pitch:{}, time:{}",
                         posRefRobot.mVal.y, horizontalDist, yawAngle, pitchAngle, time));
                     sendAllHighPriority(set_target_info_atom_v, mGroupMask, data.value().lastUpdate.time_since_epoch().count(),
                                         yawAngle, pitchAngle, true, normalSolver);
                 },
                 [this](car_predict_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(predict_success_atom, TypedIdentifier<PredictedTarget>);
                     ACTOR_EXCEPTION_PROBE();

                     auto data = BlackBoard::instance().get<PredictedTarget>(key);
                     if(!(data.has_value()))
                         return;

                     glm::dvec3 center = tf(data->center.mVal);
                     double theta = -data->yaw.mVal;
                     glm::dvec3 lVel = tf(data->linearVel.mVal);
                     double aVel = -data->angularVel.mVal;
                     double R[2] = { data->radius.first, data->radius.second };
                     double Z[2] = { data->y.first, data->y.second };

                     {
                         glm::dvec3 pos = getPos(center, R[0], theta);
                         HubLogger::watch("verticalDistance", pos.z);
                         HubLogger::watch("horizontalDistance", std::sqrt(square(pos.x) + square(pos.y)));
                     }

                     // solve and determine possible armor
                     std::optional<double> yaw, pitch;
                     for(int i = 0; i < 4; i++) {
                         double r = R[i & 1];
                         center.z = Z[i & 1];
                         double predictTime = 0;
                         for(int iterTimes = 1; iterTimes <= 5; iterTimes++) {
                             glm::dvec3 predictCenter = center + lVel * predictTime;
                             double predictTheta = theta + aVel * predictTime;
                             glm::dvec3 predictPos = getPos(predictCenter, r, predictTheta);

                             auto [airTime, yawAngle, pitchAngle] = solveWithoutAirDrag(predictPos, lVel);

                             double requiredTime = airTime + mConfig.delay + GlobalSettings::get().latency;
                             double requiredTheta = theta + aVel * requiredTime;

                             if(requiredTime - predictTime <= mConfig.sameTimeThreshold) {
                                 if(r == 0 ||
                                    abs(normalizeAngle(requiredTheta - yawAngle - glm::pi<double>())) <=
                                        glm::radians(mConfig.maxShootDeltaTheta)) {
                                     yaw = yawAngle;
                                     pitch = pitchAngle;
                                 }else{
                                    logInfo(fmt::format("AngleSolver: {}th armor do not satisfy maxShootDeltaYaw", i));
                                    HubLogger::VisualLog(fmt::format("AngleSolver: {}th armor do not satisfy maxShootDelatYaw", i));
                                 }
                                 break;
                             }
                             predictTime += mConfig.requiredTimeWeight * (requiredTime - predictTime);
                         }
                         if(yaw.has_value()) {
                             logInfo(fmt::format("AngleSolver: target id: {} yaw: {:.3f} pitch: {:.3f}", i, yaw.value(),
                                                 pitch.value()));
                             HubLogger::VisualLog(fmt::format("AngleSolver: target {}th armor yaw: {:.3f} pitch: {:.3f}", i,
                                                              yaw.value(), pitch.value()));
                             sendAllHighPriority(set_target_info_atom_v, mGroupMask, data->lastUpdate.time_since_epoch().count(),
                                                 yaw.value(), pitch.value(), true, normalSolver);
                             break;
                         }else{
                            logInfo(fmt::format("AngleSolver: solved error occurred! Four Armor do not satisfy maxShootDeltaYaw, exceed max iter times"));
                            HubLogger::VisualLog(fmt::format("AngleSolver: solved error occurred! Four Armor do not satisfy maxShootDeltaYaw, exceed max iter times"));
                         }
                         theta += (aVel < 0 ? glm::half_pi<double>() : -glm::half_pi<double>());
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(AngleSolver);
