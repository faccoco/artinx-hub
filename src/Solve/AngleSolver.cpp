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
    double lVelDiscount;
    double orietationAngle; // in degree
};

template <class Inspector>
bool inspect(Inspector& f, AngleSolverSettings& x) {
    return f.object(x).fields(
        f.field("delay", x.delay).fallback(0.0), f.field("sameTimeThreshold", x.sameTimeThreshold).fallback(0.05),
        f.field("requiredTimeWeight", x.requiredTimeWeight).fallback(1),
        f.field("maxShootDeltaTheta", x.maxShootDeltaTheta).fallback(60), f.field("lVelDiscount", x.lVelDiscount).fallback(1.0),
        f.field("orietationAngle", x.orietationAngle).fallback(20.0));
}

template <class T>
static std::optional<T> getQueueMax(const std::deque<T>& queue) {
    if(!queue.empty()) {
        T maxT = queue.front();
        for(T t : queue) {
            if(abs(t) > abs(maxT))
                maxT = t;
        }
        return maxT;
    }
    return {};
}

class AngleSolver final : public HubHelper<caf::event_based_actor, AngleSolverSettings, set_target_info_atom> {

    std::deque<double> mPastAVel;
    static double absAngleDifferece(double a, double b) {
        return std::abs(normalizeAngle(b - a));
    }
    static constexpr glm::dvec3 tf(const glm::dvec3& ori) {
        return { ori.x, -ori.z, ori.y };
    }

    static glm::dvec3 getPos(const glm::dvec3& center, double r, double theta) {
        return { center.x + r * cos(theta), center.y + r * sin(theta), center.z };
    }

    double getMaxAVel(double aVel) {
        mPastAVel.push_back(aVel);
        if(mPastAVel.size() > 3) {
            if(mPastAVel.size() > 50)
                mPastAVel.pop_front();
            return getQueueMax<double>(mPastAVel).value();
        } else
            return aVel;
    }

public:
    AngleSolver(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, std::move(name) } {}
    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
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

                auto [accessible, time, yawAngle, pitchAngle] = solveWithoutAirDrag(tfPos, tfLinearVel);
                //                     logInfo(fmt::format("x:{}, y:{}, z:{}, xVel:{}, yVel:{}, zVel:{}", tfPos.x, tfPos.y,
                //                     tfPos.z, tfLinearVel.x, tfLinearVel.y, tfLinearVel.z)); logInfo(fmt::format("time:{},
                //                     yawAngle:{}, pitch:{}", time, yawAngle, pitchAngle));
                //  HubLogger::visualLog(fmt::format(
                //      "AngleSolver: target verDist: {:.3f} horizDist: {:.3f}, solved angle yaw:{}, pitch:{}, time:{}",
                //      posRefRobot.mVal.y, horizontalDist, yawAngle, pitchAngle, time));
                if(accessible) {
                    sendAllHighPriority(set_target_info_atom_v, mGroupMask, data.value().lastUpdate.time_since_epoch().count(),
                                        yawAngle, pitchAngle, true, normalSolver);
                } else {
                    HubLogger::visualLog("AngleSolver: armor inaccessable (single armor)");
                }
            },
            [&](car_predict_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(predict_success_atom, TypedIdentifier<PredictedTarget>);
                ACTOR_EXCEPTION_PROBE();

                auto data = BlackBoard::instance().get<PredictedTarget>(key);
                if(!(data.has_value()))
                    return;

                glm::dvec3 center = tf(data->center.mVal);
                double theta = -data->yaw.mVal;

                double centerYaw = normalizeAngle(atan2(center.y, center.x) - glm::half_pi<double>());
                // glm::dvec3 lVel = tf(data->linearVel.mVal) * mConfig.lVelDiscount;
                glm::dvec3 lVel = tf(data->linearVel.mVal);

                double aVel = -data->angularVel.mVal;

                // double aVel = getMaxAVel(-data->angularVel.mVal);
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
                std::optional<int> targetArmorId;
                int armorNum = data->armorNum;
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
                                double angleDiff = absAngleDifferece(centerYaw, normalizeAngle(yawAngle-glm::half_pi<double>()));
                                if(angleDiff - glm::radians(mConfig.orietationAngle) <= 1e-6) {
                                    if(!yaw.has_value() || angleDiff < absAngleDifferece(centerYaw, yaw.value())) {
                                        yaw = yawAngle;
                                        pitch = pitchAngle;
                                        targetArmorId = i;
                                        // if(targetArmorId.value() != 0){
                                        //                         HubLogger::visualLog(fmt::format("AngleSolver: target switch to {}th armor yaw: {:.3f} pitch: {:.3f}",
                                        //              targetArmorId.value(), yaw.value(), pitch.value()));
                                        // }
										// logInfo(fmt::format(
                                        // "AngleSolver: {}th armor yawAngle diff:{:.3f} degrees satisfy oritationAngle", i,
                                        // glm::degrees(angleDiff)));
                                    }
                                } else {
                                    // logInfo(fmt::format(
                                    //     "AngleSolver: {}th armor yawAngle diff:{:.3f} degrees do not satisfy oritationAngle", i,
                                    //     glm::degrees(angleDiff)));
                                }
                            } else {
                                // logInfo(fmt::format("AngleSolver: {}th armor deltaTheta:{:.3f} do not satisfy maxShootDelatYaw",
                                //                     i, deltaTheta));
                            }
                            break;
                        }
                        predictTime += mConfig.requiredTimeWeight * (requiredTime - predictTime);
                    }
                    theta += (aVel < 0 ? glm::two_pi<double>() / armorNum : -glm::two_pi<double>() / armorNum);
                }
                if(yaw.has_value()) {

                    HubLogger::visualLog(fmt::format("AngleSolver: target {}th armor yaw: {:.3f} pitch: {:.3f}",
                                                     targetArmorId.value(), yaw.value(), pitch.value()));
                    sendAllHighPriority(set_target_info_atom_v, mGroupMask, data->lastUpdate.time_since_epoch().count(),
                                        yaw.value(), pitch.value(), true, normalSolver);
                } else {
                    // logInfo(fmt::format("AngleSolver: {}th armor exceed max iter times or not satisfy maxShootDeltaYaw",
                    // i));
                }
                // logInfo("AngleSolver: solve failed! Four Armor do not satisfy maxShootDeltaYaw");
            },
        };
    }
};

HUB_REGISTER_CLASS(AngleSolver);