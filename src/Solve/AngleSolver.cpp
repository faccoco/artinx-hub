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
    int maxIterTimes;
    double requiredTimeWeight;
    double maxShootDeltaTheta;  // in degree
};

template <class Inspector>
bool inspect(Inspector& f, AngleSolverSettings& x) {
    return f.object(x).fields(f.field("delay", x.delay), f.field("sameTimeThreshold", x.sameTimeThreshold).fallback(0.05),
                              f.field("maxIterTimes", x.maxIterTimes).fallback(5),
                              f.field("requiredTimeWeight", x.requiredTimeWeight).fallback(1),
                              f.field("maxShootDeltaTheta", x.maxShootDeltaTheta).fallback(60));
}

class AngleSolver final : public HubHelper<caf::event_based_actor, AngleSolverSettings, set_target_info_atom> {
    double maxShootDeltaTheta = glm::radians(mConfig.maxShootDeltaTheta);

    static constexpr glm::dvec3 tf(const glm::dvec3& ori) {
        return { ori.x, -ori.z, ori.y };
    }

    glm::dvec3 getPos(const glm::dvec3& center, double r, double theta) {
        return { center.x + r * cos(theta), center.y + r * sin(theta), center.z };
    }

public:
    AngleSolver(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}

    /*
        // tuple[time,yawAngle,pitchAngle]
        std::tuple<double, double, double> solveWithAirDrag(glm::dvec3 targetPos, glm::dvec3 targetVel) {
            double x = targetPos.x, y = targetPos.y, z = targetPos.z;
            double vx = targetVel.x, vy = targetVel.y, vz = targetVel.z;
            double k = 2 * bulletMass / (dragCoefficient * airDensity * bulletRadius * bulletRadius * glm::pi<double>());
            double tmpx = glm::sqrt(k / (k - 2 * x)), tmpy = glm::sqrt(k / (k - 2 * y));
            double a = g * g / 4;
            double b = g * vz;
            double c = vx * vx * tmpx * tmpx * tmpx + vy * vy * tmpy * tmpy * tmpy + g * z + vz * vz - bulletSpeed *
       bulletSpeed; double d = 2 * k * (vx * (tmpx - 1) + vy * (tmpy - 1)) + 2 * vz * z; double e = 2 * k * (k - x - k / tmpx
       + k - y - k / tmpy) + z * z;

            double t = ferrari(a, b, c, d, e);
            double v0x = k * (1 - glm::sqrt(1 - 2 * x / k - 2 * vx * t / k)) / t;
            double v0y = k * (1 - glm::sqrt(1 - 2 * y / k - 2 * vy * t / k)) / t;
            double v0z = g * t / 2 + vz + z / t;

            double pitchAngle = std::asin(v0z / bulletSpeed);
            double yawAngle = std::atan2(v0y, v0x) - glm::half_pi<double>();

            return std::make_tuple(t, yawAngle, pitchAngle);
        }
     */
    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](predict_success_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(predict_success_atom, TypedIdentifier<PredictedTarget>);
                ACTOR_EXCEPTION_PROBE();

                auto data = BlackBoard::instance().get<PredictedTarget>(key);
                if(!(data.has_value()))
                    return;

                glm::dvec3 center = tf(data->center.mVal);
                double theta = -data->theta.mVal;
                glm::dvec3 lVel = tf(data->lVel.mVal);
                double aVel = -data->aVel.mVal;
                double R[2] = { data->radius.first, data->radius.second };
                double Z[2] = { data->y.first, data->y.second };
                double shootDelayTime = GlobalSettings::get().shootDelayTime;

                {
                    glm::dvec3 pos = getPos(center, R[0], theta);
                    //                    logInfo(fmt::format("AngleSolver: tfpos: {:.3f} {:.3f} {:.3f}", pos.x, pos.y, pos.z));
                    // HubLogger::watch("x", pos.x);
                    // HubLogger::watch("y", pos.y);
                    // HubLogger::watch("z", pos.z);
                    HubLogger::watch("vertical distance", pos.z);
                    HubLogger::watch("horizontal distance", std::sqrt(square(pos.x) + square(pos.y)));
                }

                // solve and determine possible armor
                std::optional<double> yaw, pitch;
                for(int i = 0; i < 4; i++) {
                    double r = R[i & 1];
                    center.z = Z[i & 1];
                    double predictTime = 0;
                    for(int iterTimes = 1; iterTimes <= mConfig.maxIterTimes; iterTimes++) {
                        glm::dvec3 predictCenter = center + lVel * predictTime;
                        double predictTheta = theta + aVel * predictTime;
                        glm::dvec3 predictPos = getPos(predictCenter, r, predictTheta);

                        auto [airTime, yawAngle, pitchAngle] = solveWithoutAirDrag(predictPos, lVel);

                        double requiredTime = airTime + mConfig.delay + shootDelayTime;
                        //                        glm::dvec3 requiredCenter = center + lVel * requiredTime;
                        double requiredTheta = theta + aVel * requiredTime;
                        //                        glm::dvec3 requiredPos = getPos(requiredCenter, r, requiredTheta);

                        if(requiredTime - predictTime <= mConfig.sameTimeThreshold) {
                            // logInfo(fmt::format("AngleSolver: iter times: {}", iterTimes));
                            // logInfo(
                            //     fmt::format("AngleSolver: yaw: {} theta: {} delta theta : {}", glm::degrees(yawAngle),
                            //                 glm::degrees(normalizeAngle(requiredTheta)),
                            //                 glm::degrees(shortestAngularDistance(requiredTheta, yawAngle +
                            //                 glm::pi<double>()))));
                            if(r == 0 ||
                               abs(shortestAngularDistance(requiredTheta, yawAngle + glm::pi<double>())) <= maxShootDeltaTheta) {
                                yaw = yawAngle;
                                pitch = pitchAngle;
                                //                                logInfo(fmt::format("AngleSolver: requiredPos: {:.3f} {:.3f}
                                //                                {:.3f}", requiredPos.x,
                                //                                                    requiredPos.y, requiredPos.z));
                            } else {
                                //                                logInfo("AngleSolver: bad theta, switch to next armor");
                            }
                            break;
                        }
                        predictTime += mConfig.requiredTimeWeight * (requiredTime - predictTime);
                    }
                    if(yaw.has_value()) {
                        logInfo(
                            fmt::format("AngleSolver: target id: {} yaw: {:.3f} pitch: {:.3f}", i, yaw.value(), pitch.value()));
                        sendAllHighPriority(set_target_info_atom_v, mGroupMask, data->lastUpdate.time_since_epoch().count(),
                                            yaw.value(), pitch.value(), true, normalSolver);
                        break;
                    } else {
                        //                        logInfo("AngleSolver: iteration failed or bad theta");
                    }
                    theta += (aVel < 0 ? glm::half_pi<double>() : -glm::half_pi<double>());
                }
            },
        };
    }
};

HUB_REGISTER_CLASS(AngleSolver);
