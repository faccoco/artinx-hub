#define GLM_ENABLE_EXPERIMENTAL
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

#include "SuppressWarningEnd.hpp"
#include <caf/event_based_actor.hpp>
#include <cmath>
#include <fmt/core.h>
#include <fmt/format.h>
#include <glm/fwd.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/string_cast.hpp>
#include <list>
#include <magic_enum.hpp>
#include <optional>
#include <vector>

struct AngleSolverSettings final {
    bool debugView;
    double delay;
    bool gimbalFixed;
    bool enableOrietationAngleLimit;
    double sameTimeThreshold;
    double requiredTimeWeight;
    double maxShootDeltaTheta;  // in degree
    double lVelDiscount;
    double orietationAngle;  // in degree
    double latencyThreshold;
    double aVelThreshold;
};

template <class Inspector>
bool inspect(Inspector& f, AngleSolverSettings& x) {
    return f.object(x).fields(
        f.field("debugView", x.debugView).fallback(false), f.field("delay", x.delay).fallback(0.0),
        f.field("gimbalFixed", x.gimbalFixed).fallback(false), f.field("sameTimeThreshold", x.sameTimeThreshold).fallback(0.05),
        f.field("requiredTimeWeight", x.requiredTimeWeight).fallback(1),
        f.field("maxShootDeltaTheta", x.maxShootDeltaTheta).fallback(60), f.field("lVelDiscount", x.lVelDiscount).fallback(1.0),
        f.field("orietationAngle", x.orietationAngle).fallback(37), f.field("latencyThreshold", x.latencyThreshold).fallback(500),
        f.field("enableOrietationAngleLimit", x.enableOrietationAngleLimit).fallback(false),
        f.field("aVelThreshold", x.aVelThreshold).fallback(9.0));
}

struct CandidateTarget final {
    double yawAngle;
    double pitchAngle;
    double diffAngle;
    double r;
    double height;
    glm::dvec3 pos;
    double reachTime;
};

class AngleSolver final
    : public HubHelper<caf::event_based_actor, AngleSolverSettings, set_target_info_atom, angle_solver_view_atom> {
    CameraFrame mFrame;
    bool frameInit;
    Identifier mKey;
    std::deque<double> mPastAVel;
    std::list<double> latency;
    static double absAngleDifferece(double a, double b) {
        return std::abs(normalizeAngle(b - a));
    }
    TimePoint latestReceived;

    static constexpr glm::dvec3 tf(const glm::dvec3& ori) {
        return { ori.x, -ori.z, ori.y };
    }

    static constexpr glm::dvec3 inverseTf(const glm::dvec3& ori) {
        return { ori.x, ori.z, -ori.y };
    }

    static glm::dvec3 getPos(const glm::dvec3& center, double r, double theta) {
        return { center.x + r * cos(theta), center.y + r * sin(theta), center.z };
    }

    static double getAvglatency(std::list<double>& latency, double threshold) {
        double addLatency = 0;
        if(latency.size() < 20) {
            latency.push_back(GlobalSettings::get().shootDelayTime);
        } else {
            latency.pop_front();
            latency.push_back(GlobalSettings::get().shootDelayTime);
        }

        int count = 0;
        for(double a : latency) {
            if(a < threshold) {
                addLatency += a;
                count++;
            }
        }
        double delay = addLatency / count;
        HubLogger::watch("avgShootDelay", delay);
        return delay;
    }

    void targetView(const PredictedTarget& target, const double reachTime) {
#ifndef ARTINXHUB_DEBUG
//        return;
#endif
        auto theta = -target.yaw.mVal;
        auto aVel = std::abs(target.angularVel.mVal) > 1 ? -target.angularVel.mVal : 0;
        auto lVel = target.linearVel.mVal;

        auto predictCarCenter = target.center.mVal + lVel * reachTime;
        auto predictTheta = theta + aVel * reachTime;

        std::vector<cv::Point3d> pointsList;
        std::vector<cv::Point3d> pointsListPred;
        for(int i = 0; i < target.armorNum; i++) {
            auto r = i % 2 ? target.radius.second : target.radius.first;
            auto y = i % 2 ? target.y.second : target.y.first;
            auto armorTheta = theta + (i * (2 * glm::pi<double>() / target.armorNum));
            auto armorThetaPred = predictTheta + (i * (2 * glm::pi<double>() / target.armorNum));

            // armorPos
            Point<UnitType::Distance, FrameOfRef::Robot> armorPos = inverseTf(getPos(tf(target.center.mVal), r, armorTheta));
            armorPos.mVal.y = y;
            auto posRefCam = Point<UnitType::Distance, FrameOfRef::Camera>{ target.tfRobot2Camera(armorPos).mVal };
            pointsList.emplace_back(posRefCam.mVal.x, -posRefCam.mVal.y, -posRefCam.mVal.z);

            // predicted armorPos
            Point<UnitType::Distance, FrameOfRef::Robot> predictArmorPos =
                inverseTf(getPos(tf(predictCarCenter), r, armorThetaPred));
            predictArmorPos.mVal.y = y;
            auto predictPosRefCam = Point<UnitType::Distance, FrameOfRef::Camera>{ target.tfRobot2Camera(predictArmorPos).mVal };
            pointsListPred.emplace_back(predictPosRefCam.mVal.x, -predictPosRefCam.mVal.y, -predictPosRefCam.mVal.z);
        }

        Point<UnitType::Distance, FrameOfRef::Robot> carCenter = target.center.mVal;
        carCenter.mVal.y = (target.y.first + target.y.second) / 2;
        auto carCenterRefCam = Point<UnitType::Distance, FrameOfRef::Camera>{ target.tfRobot2Camera(carCenter).mVal };

        Point<UnitType::Distance, FrameOfRef::Robot> carCenterPred = predictCarCenter;
        carCenterPred.mVal.y = (target.y.first + target.y.second) / 2 + lVel.y * reachTime;
        auto carCenterPredRefCam = Point<UnitType::Distance, FrameOfRef::Camera>{ target.tfRobot2Camera(carCenterPred).mVal };

        pointsList.emplace_back(carCenterRefCam.mVal.x, -carCenterRefCam.mVal.y, -carCenterRefCam.mVal.z);
        pointsListPred.emplace_back(carCenterPredRefCam.mVal.x, -carCenterPredRefCam.mVal.y, -carCenterPredRefCam.mVal.z);

        std::vector<cv::Point2d> imagPoints;
        std::vector<cv::Point2d> imagPointsPred;

        if (frameInit) {
            cv::projectPoints(pointsList, cv::Vec3d{ 0, 0, 0 }, cv::Vec3d{ 0, 0, 0 }, mFrame.info.cameraMatrix,
                              mFrame.info.distCoefficients, imagPoints);
            cv::projectPoints(pointsListPred, cv::Vec3d{ 0, 0, 0 }, cv::Vec3d{ 0, 0, 0 }, mFrame.info.cameraMatrix,
                              mFrame.info.distCoefficients, imagPointsPred);
        }else {
            logWarning("AngleSolver Visualization: frame not initialized");
        }

        // draw armor corners
        double armorPitch = glm::radians(-15.0f);
        double armorYaw = target.yaw.mVal - glm::half_pi<double>();

        auto rotationMatrix = glm::rotate(glm::rotate(glm::identity<glm::dmat4>(), -armorPitch, glm::dvec3(1, 0, 0)), armorYaw, glm::dvec3(0, 1, 0));
        auto armorPos = inverseTf(getPos(tf(target.center.mVal), target.radius.first, -target.yaw.mVal));
        auto transformMatrix = glm::translate(glm::identity<glm::dmat4>(), -armorPos);
        auto mTfArmor2Robot = Transform<FrameOfRef::Robot, FrameOfRef::Armor, true>{ rotationMatrix * transformMatrix };
        auto mTfArmor2Camera = combine(mTfArmor2Robot.invTransformObj(), target.tfRobot2Camera);

        std::vector<cv::Point3d> armorCorner;
        for (const auto& point : target.armorType == ArmorType::Large ? mObjectPointsLarge : mObjectPointsSmall){
            auto posArmor = Point<UnitType::Distance, FrameOfRef::Armor>{ glm::dvec3 { point.x, point.y, point.z } };
            auto posCamera = mTfArmor2Camera(posArmor);
            armorCorner.emplace_back(posCamera.mVal.x, -posCamera.mVal.y, -posCamera.mVal.z);
        }

        std::vector<cv::Point2d> imagPointsArmor;
        if (frameInit) {
            cv::projectPoints(armorCorner, cv::Vec3d{ 0, 0, 0 }, cv::Vec3d{ 0, 0, 0 }, mFrame.info.cameraMatrix,
                              mFrame.info.distCoefficients, imagPointsArmor);
        }else {
            logWarning("AngleSolver Visualization: frame not initialized");
        }

        ProjectedTarget res;
        res.lastUpdate = target.lastUpdate;
        res.armorCorners = imagPointsArmor;
        res.projectedPoints = std::make_pair(imagPoints, imagPointsPred);
        sendAll(angle_solver_view_atom_v,
                BlackBoard::instance().updateSync<ProjectedTarget>(Identifier{ mKey.val }, std::move(res)));
    }

    static CandidateTarget chooseTarget(const std::vector<CandidateTarget>& candTargets) {
        std::sort(candTargets.begin(), candTargets.end(),
                  [](const CandidateTarget& a, const CandidateTarget& b) { return a.diffAngle < b.diffAngle; });

        return candTargets[0];
    }

public:
    AngleSolver(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, std::move(name) }, mKey{ generateKey(this) } {
        latestReceived = TimePoint::min();
        frameInit = false;
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
                // HubLogger::watch("verticalDistance", posRefRobot.mVal.y);
                // HubLogger::watch("horizontalDistance", horizontalDist);

                //(forward:+y,right:+x)
                glm::dvec3 tfPos = tf(posRefRobot.mVal);
                glm::dvec3 tfLinearVel = tf(linearVel.mVal);

                const auto delayTime = mConfig.delay + GlobalSettings::get().latency;
                tfPos = { tfPos.x + delayTime * tfLinearVel.x, tfPos.y + delayTime * tfLinearVel.y,
                          tfPos.z + delayTime * tfLinearVel.z };

                auto [accessible, time, yawAngle, pitchAngle] = solveWithoutAirDrag(tfPos, tfLinearVel);

                //                  logInfo(fmt::format("x:{}, y:{}, z:{}, xVel:{}, yVel:{}, zVel:{}", tfPos.x, tfPos.y,
                //                     tfPos.z, tfLinearVel.x, tfLinearVel.y, tfLinearVel.z)); logInfo(fmt::format("time:{},
                //                     yawAngle:{}, pitch:{}", time, yawAngle, pitchAngle));
                //                  HubLogger::visualLog(fmt::format(
                //                      "AngleSolver: target verDist: {:.3f} horizDist: {:.3f}, solved angle yaw:{}, pitch:{},
                //                      time:{}", posRefRobot.mVal.y, horizontalDist, yawAngle, pitchAngle, time));

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
                if(!frameInit) {
                    mFrame = data->frame;
                    frameInit = true;
                }

                // check pkg order
                if(data->lastUpdate.time_since_epoch().count() > latestReceived.time_since_epoch().count()) {
                    latestReceived = data->lastUpdate;
                } else {
                    // logInfo("angle solver pkg order wrong! ignore wrong order");
                    return;
                }
                glm::dvec3 center = tf(data->center.mVal);
                double theta = -data->yaw.mVal;
                double centerYaw = normalizeAngle(atan2(center.y, center.x) - glm::half_pi<double>());
                glm::dvec3 lVel = tf(data->linearVel.mVal);

                double aVel = std::abs(data->angularVel.mVal) > 1 ? -data->angularVel.mVal : 0;

                // HubLogger::watch("CenterYaw", centerYaw);
                // HubLogger::watch("AngleVelRefRobot", aVel);

                double R[2] = { data->radius.first, data->radius.second };
                double Z[2] = { data->y.first, data->y.second };

                {
                    // glm::dvec3 pos = getPos(center, R[0], theta);
                    // HubLogger::watch("verticalDistance", pos.z);
                    // HubLogger::watch("horizontalDistance", std::sqrt(square(pos.x) + square(pos.y)));
                }

                // solve and determine possible armor
                int armorNum = data->armorNum;
                std::vector<CandidateTarget> candTargets;
                double avgLatency = 0.0;
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

                        double delay = mConfig.gimbalFixed ? getAvglatency(latency, mConfig.latencyThreshold) / 1000 : mConfig.delay;
                        double requiredTime = airTime + delay + GlobalSettings::get().latency;
                        double requiredTheta = theta + aVel * requiredTime;

                        if(requiredTime - predictTime <= mConfig.sameTimeThreshold) {
                            double deltaTheta = normalizeAngle(requiredTheta - yawAngle - glm::pi<double>());
                            if(r == 0 || std::abs(deltaTheta) <= glm::radians(mConfig.maxShootDeltaTheta)) {
                                double angleDiff =
                                    absAngleDifferece(centerYaw, normalizeAngle(yawAngle - glm::half_pi<double>()));
                                CandidateTarget candTarget{};
                                candTarget.yawAngle = yawAngle;
                                candTarget.pitchAngle = pitchAngle;
                                candTarget.diffAngle = angleDiff;
                                candTarget.r = r;
                                candTarget.height = center.z;
                                candTarget.pos = predictPos;
                                candTarget.reachTime = requiredTime;
                                candTargets.push_back(candTarget);
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
                    std::optional<double> yaw, pitch;
                    glm::dvec3 targetPos;
                    bool fire = false;

                    CandidateTarget selectedTarget = chooseTarget(candTargets);

                    yaw = selectedTarget.yawAngle;
                    pitch = selectedTarget.pitchAngle;
                    targetPos = selectedTarget.pos;
                    if(mConfig.enableOrietationAngleLimit) {
                        // TODO(12012710): to be discuss
                        if(aVel > mConfig.aVelThreshold && selectedTarget.diffAngle > glm::radians(mConfig.orietationAngle)) {
                            yaw.reset();
                            pitch.reset();
                        }
                    }
                    if(mConfig.gimbalFixed) {
                        double r = selectedTarget.r;
                        center.z = selectedTarget.height;
                        auto armorFaced = getPos(
                            center, r, normalizeAngle(glm::half_pi<double>() + centerYaw));  // armor_yaw - center_yaw - half_pi
                        auto [accessible, airTime, yawAngle, pitchAngle] = solveWithoutAirDrag(armorFaced, lVel);
                        if(accessible) {
                            fire = glm::degrees(selectedTarget.diffAngle) < mConfig.orietationAngle;
                            // HubLogger::watch("diffAngle", selectedTarget.diffAngle);
                            yaw = yawAngle;
                            pitch = pitchAngle;
                            targetPos = armorFaced;
                        }
                    }
                    SelectedTargetInfo res;
                    if(yaw.has_value()) {
                        res.yawAngle = yaw.value();
                        res.pitchAngle = pitch.value();
                        res.solveType = normalSolver;
                        res.isFire = fire;
                        res.targetPos = targetPos;
                        res.lastUpdate = data.value().lastUpdate;
                        res.targetType = data.value().robotType;
                        res.armorType = data.value().armorType;
                        sendAll(set_target_info_atom_v,
                                BlackBoard::instance().updateSync<SelectedTargetInfo>(Identifier{ mKey.val }, res));
                        HubLogger::visualLog(fmt::format("AngleSolver: target {}th armor yaw: {:.3f} pitch: {:.3f}", 0,
                                                         yaw.value(), pitch.value()));
                    }
                    HubLogger::watch("fire", fire);

                    if(mConfig.gimbalFixed) {
                        HubLogger::visualLog(fmt::format("AngleSolver: Shootdelay time is {}, avg is {}",
                                                         GlobalSettings::get().shootDelayTime, avgLatency));
                    }

                    if(mConfig.debugView) {
                        targetView(data.value(), selectedTarget.reachTime - mConfig.delay);
                    }
                    return;
                }
            }
        };
    }
};

HUB_REGISTER_CLASS(AngleSolver);