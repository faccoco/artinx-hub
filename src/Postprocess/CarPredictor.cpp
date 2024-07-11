// Ref: https://github.com/chenjunnn/rm_auto_aim

#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "EKF.hpp"
#include "ExceptionProbe.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"

#include "SuppressWarningBegin.hpp"
#include "Utility.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <caf/event_based_actor.hpp>
#include <cmath>
#include <fmt/core.h>
#include <glm/ext.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/quaternion_common.hpp>
#include <glm/ext/scalar_constants.hpp>
#include <glm/fwd.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/matrix.hpp>
#include <glm/trigonometric.hpp>
#include <iostream>
#include <magic_enum.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>

struct CarPredictorSettings final {
    bool enablePredictor;
    bool debugView;
    bool enableFixYaw;
    double fixYawThresh;
    double maxMatchDist;
    double maxMatchYaw;
    int trackingThreshold;
    int lostThreshold;
    double sigma2Qxyz;  // Process noise variance of xyz
    double sigma2Qyaw;  // Process noise variance of yaw
    double sigma2QR;    // Process noise variance of r
    double Rxyz;        // Measurement covariance matrix factor of xyz
    double Ryaw;        // Measurement covariance matrix factor of yaw
};

template <class Inspector>
bool inspect(Inspector& f, CarPredictorSettings& x) {
    return f.object(x).fields(
        f.field("debugView", x.debugView).fallback(false), f.field("fixYawThresh", x.fixYawThresh).fallback(90.0),
        f.field("enableFixYaw", x.enableFixYaw).fallback(false), f.field("enablePredictor", x.enablePredictor),
        f.field("maxMatchDist", x.maxMatchDist).fallback(0.4), f.field("maxMatchYaw", x.maxMatchYaw).fallback(0.3),
        f.field("trackingThreshold", x.trackingThreshold).fallback(5), f.field("lostThreshold", x.lostThreshold).fallback(5),
        f.field("sigma2Qxyz", x.sigma2Qxyz).fallback(20.0), f.field("sigma2Qyaw", x.sigma2Qyaw).fallback(100.0),
        f.field("sigma2QR", x.sigma2QR).fallback(800.0), f.field("Rxyz", x.Rxyz).fallback(0.05),
        f.field("Ryaw", x.Ryaw).fallback(0.02));
}

class CarPredictor final
    : public HubHelper<caf::event_based_actor, CarPredictorSettings, car_predict_atom, car_predict_view_atom> {
    Identifier mKey, mIMUKey;

    enum class TrackingState {
        LOST,
        DETECTING,
        TRACKING,
        TEMP_LOST,
    };

    enum class YawFixMode {
        NONE,
        SHORT_EDGE_ANGLE,
        LONG_EDGE_ANGLE,
        POLYGON_DIFF
    };

    ExtendedKalmanFilter mEKF;
    struct TrackedArmor {
        TimePoint lastUpdate;
        Eigen::VectorXd state;
        double yaw;
        RobotType id;
        TrackingState trackingState;
        int armorNum;
        ArmorType armorType;
        std::vector<cv::Point> armorPoints;
    } mTrackedArmor;
    double mLastY = 0.0, mLastR = 0.2;
    int mDetectCount = 0, mLostCount = 0;
    double mDt = 0.01;
    CameraFrame mFrame;
    bool frameInit = false;

    static double euclideanDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
        cv::Point diff = p1 - p2;
        return cv::sqrt(diff.x * diff.x + diff.y * diff.y);
    }

    Transform<FrameOfRef::Camera, FrameOfRef::Robot, true> mTfCamera2Robot;

    glm::dvec3 getArmorPos(const DetectedTarget& armor) {
        return mTfCamera2Robot(armor.center).mVal;
    }

    double getArmorYaw(const DetectedTarget& armor) {
        auto rmat = combine(mTfCamera2Robot, armor.rmat);
        auto yaw = normalizeAngle(-atan2(rmat.raw()[2][0], rmat.raw()[2][2]) - glm::half_pi<double>());
        return yaw;
    }

    void debugView(std::vector<cv::Point2d>& armorPoints) {
        ProjectedArmor projectedArmor;
        projectedArmor.armorCorners = armorPoints;
        projectedArmor.lastUpdate = mTrackedArmor.lastUpdate;
        sendAll(car_predict_view_atom_v, BlackBoard::instance().updateSync(mKey, std::move(projectedArmor)));
    }

    double fixArmorYaw(const DetectedTarget& armor, double yaw) {

        if(!mConfig.enableFixYaw) {
            return yaw;
        }
        YawFixMode fixMode = YawFixMode::NONE;

        double yawFace =
            std::abs(glm::degrees(normalizeAngle(-atan2(armor.rmat.raw()[2][0], armor.rmat.raw()[2][2]) - glm::pi<double>())));


        if(std::abs(yawFace) < mConfig.fixYawThresh) {
            fixMode = YawFixMode::SHORT_EDGE_ANGLE;
        }

        HubLogger::watch("fixMode", static_cast<int>(fixMode));
        HubLogger::watch("yawFace", yawFace);

        if(fixMode == YawFixMode::NONE) {
            return yaw;
        }

        double gap = 3 / 180.0 * CV_PI;
        int cnt = 0;
        double left = yaw - CV_PI / 8, right = yaw + CV_PI / 8;
        // double initLoss = calLoss(armor, yaw, YawFixMode::POLYGON_DIFF).value();
        while(right - left > gap && cnt < 5) {
            double mid = (left + right) / 2;
            auto lossLeft = calLoss(armor, (left + right) / 2 - gap, fixMode);
            auto lossRight = calLoss(armor, (left + right) / 2 + gap, fixMode);
            if((!lossLeft.has_value() || !lossRight.has_value()) || 
                (lossLeft.value() == 0.0 && lossRight.value() == 0.0)) {
                return yaw;
            }
            if(lossLeft.value() < lossRight.value()) {
                right = mid;
            } else {
                left = mid;
            }
            cnt++;
        }
        // double finalLoss = calLoss(armor, (left + right) / 2, YawFixMode::POLYGON_DIFF).value();
        // if(finalLoss > initLoss && yawFace < 15.0f) {
        //     return yaw;
        // }
        return (left + right) / 2;
    }

    std::optional<double> calLoss(const DetectedTarget& armor, double yaw, YawFixMode fixMode) {

        auto projectedPoints = reproject(armor, yaw);
        if(!projectedPoints.has_value()) {
            return {};
        }

        // calculate loss
        double armorLoss = 0.0;
        switch(fixMode) {
            case YawFixMode::SHORT_EDGE_ANGLE: {
                //      1. short edge angle
                auto edge1 = projectedPoints.value()[0] - projectedPoints.value()[1];
                auto edge2 = projectedPoints.value()[3] - projectedPoints.value()[2];

                auto angle1 = atan2(edge1.y, edge1.x);
                auto angle2 = atan2(edge2.y, edge2.x);

                auto imageEdge1 = armor.armorPoints[0] - armor.armorPoints[1];
                auto imageEdge2 = armor.armorPoints[3] - armor.armorPoints[2];

                auto imageAngle1 = atan2(imageEdge1.y, imageEdge1.x);
                auto imageAngle2 = atan2(imageEdge2.y, imageEdge2.x);

                armorLoss += std::abs(angle1 - imageAngle1) + std::abs(angle2 - imageAngle2);
                break;
            }
            case YawFixMode::LONG_EDGE_ANGLE: {
                //      2. long edge angle
                auto edge1 = projectedPoints.value()[0] - projectedPoints.value()[3];
                auto edge2 = projectedPoints.value()[1] - projectedPoints.value()[2];

                auto angle1 = atan2(edge1.y, edge1.x);
                auto angle2 = atan2(edge2.y, edge2.x);

                auto imageEdge1 = armor.armorPoints[0] - armor.armorPoints[3];
                auto imageEdge2 = armor.armorPoints[1] - armor.armorPoints[2];

                auto imageAngle1 = atan2(imageEdge1.y, imageEdge1.x);
                auto imageAngle2 = atan2(imageEdge2.y, imageEdge2.x);

                armorLoss += std::abs(angle1 - imageAngle1) + std::abs(angle2 - imageAngle2);
                break;
            }
            case YawFixMode::POLYGON_DIFF: {
                //      3. polygon difference
                std::vector<cv::Point2f> projectedPointsFloat(projectedPoints.value().begin(), projectedPoints.value().end());
                double area1 = cv::contourArea(projectedPointsFloat);
                double area2 = cv::contourArea(armor.armorPoints);

                std::vector<cv::Point2f> intersection;
                cv::intersectConvexConvex(projectedPointsFloat, armor.armorPoints, intersection);
                armorLoss = std::fabs(cv::contourArea(intersection) - (area1 + area2) / 2);
                break;
            }
            default: {
                logWarning("CarPrediction: fixMode not supported");
                break;
            }
        }

        HubLogger::watch("armorLoss", armorLoss);
        return armorLoss;
    };

    std::optional<std::vector<cv::Point2d>> reproject(const DetectedTarget& armor, double yaw) {
        double armorPitch = glm::radians(-15.0f);
        double armorYaw = yaw - glm::half_pi<double>();

        auto rotationMatrix = glm::rotate(glm::rotate(glm::identity<glm::dmat4>(), -armorPitch, glm::dvec3(1, 0, 0)), armorYaw,
                                          glm::dvec3(0, 1, 0));
        auto armorPos = mTfCamera2Robot(armor.center).mVal;

        auto transformMatrix = glm::translate(glm::identity<glm::dmat4>(), -armorPos);
        auto mTfArmor2Robot = Transform<FrameOfRef::Robot, FrameOfRef::Armor, true>{ rotationMatrix * transformMatrix };
        auto mTfArmor2Camera = combine(mTfArmor2Robot.invTransformObj(), mTfCamera2Robot.invTransformObj());

        std::vector<cv::Point3d> armorCorner;
        for(const auto& point : armor.type == ArmorType::Large ? mObjectPointsLarge : mObjectPointsSmall) {
            auto posArmor = Point<UnitType::Distance, FrameOfRef::Armor>{ glm::dvec3{ point.x, point.y, point.z } };
            auto posCamera = mTfArmor2Camera(posArmor);
            armorCorner.emplace_back(posCamera.mVal.x, -posCamera.mVal.y, -posCamera.mVal.z);
        }

        std::vector<cv::Point2d> imagPointsArmor;
        if(frameInit) {
            cv::projectPoints(armorCorner, cv::Vec3d{ 0, 0, 0 }, cv::Vec3d{ 0, 0, 0 }, mFrame.info.cameraMatrix,
                              mFrame.info.distCoefficients, imagPointsArmor);
        } else {
            logWarning("CarPrediction: frame not initialized");
            return {};
        }
        return imagPointsArmor;
    }

    void setArmorYaw(double yaw) {
        // Make yaw change continuous
        mTrackedArmor.yaw = mTrackedArmor.yaw + normalizeAngle(yaw - mTrackedArmor.yaw);
    }

    glm::dvec3 getArmorPosFromState(const Eigen::VectorXd& x) {
        // Calculate predicted position of the current armor
        double xc = x(0), ya = x(1), zc = x(2);
        double yaw = x(3), r = x(8);
        double xa = xc + r * cos(yaw);
        double za = zc + r * sin(yaw);
        return glm::dvec3{ xa, ya, za };
    }

    bool handleArmorJump(const glm::dvec3& targetPos, double targetYaw) {
        bool match = false;
        setArmorYaw(targetYaw);
        double yaw = mTrackedArmor.yaw;
        auto deltayaw = std::fabs(yaw - mTrackedArmor.state(3));
        if(std::fabs(yaw - mTrackedArmor.state(3)) > mConfig.maxMatchYaw) {
            match = true;
            mLastY = mTrackedArmor.state(1);
            mTrackedArmor.state(1) = targetPos.y;
            mTrackedArmor.state(3) = yaw;
            std::swap(mTrackedArmor.state(8), mLastR);
            logInfo(
                fmt::format("ArmorPredictor: Armor may experience a jump. Change Yaw, Y and R, delta yaw is {:.5f}", deltayaw));
            HubLogger::visualLog(
                fmt::format("EKF Armor may experience a jump. Change Yaw, Y and R. delta yaw is {:.5f}", deltayaw));
        }
        auto dist = glm::distance(targetPos, getArmorPosFromState(mTrackedArmor.state));

        // ignore R distance
        if(dist > mConfig.maxMatchDist + 0.2) {
            match = false;
            mTrackedArmor.state(0) = targetPos.x - mTrackedArmor.state(8) * cos(yaw);
            mTrackedArmor.state(2) = targetPos.z - mTrackedArmor.state(8) * sin(yaw);
            mTrackedArmor.state(4) = 0;
            mTrackedArmor.state(5) = 0;
            mTrackedArmor.state(6) = 0;
            logInfo(fmt::format("ArmorPredictor: The same Armor match distance too far. State wrong, reset EKF. Dist is {:.5f}",
                                dist));
            HubLogger::visualLog(
                fmt::format("ArmorPredictor: The same Armor match distance {} too far. State wrong, reset EKF", dist));
        }
        mEKF.setState(mTrackedArmor.state);
        return match;
    }

    void handleTrackingState(bool matched) {
        switch(mTrackedArmor.trackingState) {
            case TrackingState::DETECTING: {
                if(matched) {
                    mDetectCount++;
                    if(mDetectCount > mConfig.trackingThreshold) {
                        mDetectCount = 0;
                        mTrackedArmor.trackingState = TrackingState::TRACKING;
                    }
                } else {
                    mDetectCount = 0;
                    mTrackedArmor.trackingState = TrackingState::LOST;
                }
                break;
            }
            case TrackingState::TRACKING: {
                if(!matched) {
                    mTrackedArmor.trackingState = TrackingState::TEMP_LOST;
                    mLostCount++;
                }
                break;
            }
            case TrackingState::TEMP_LOST: {
                if(!matched) {
                    mLostCount++;
                    if(mLostCount > mConfig.lostThreshold) {
                        mLostCount = 0;
                        mTrackedArmor.trackingState = TrackingState::LOST;
                    }
                } else {
                    mTrackedArmor.trackingState = TrackingState::TRACKING;
                    mLostCount = 0;
                }
                break;
            }
            default:
                break;
        }
        // logInfo(fmt::format("ArmorPredictor: tracking state: {}, detectCount: {}, lostCount: {}",
        //                     magic_enum::enum_name(mTrackedArmor.trackingState), mDetectCount, mLostCount));
        HubLogger::visualLog(fmt::format("ArmorPredictor: tracking state: {}, detectCount: {}, lostCount: {}",
                                         magic_enum::enum_name(mTrackedArmor.trackingState), mDetectCount, mLostCount));
    }

    void init(const DetectedTarget& armor) {
        mTrackedArmor.yaw = 0;
        setArmorYaw(getArmorYaw(armor));
        // Set initial position at 0.2m behind the target
        double r = 0.2, yaw = mTrackedArmor.yaw;
        auto p = getArmorPos(armor);
        double x = p.x - r * cos(yaw);
        double y = p.y;
        double z = p.z - r * sin(yaw);
        mLastY = y, mLastR = r;
        mTrackedArmor.state << x, y, z, yaw, 0, 0, 0, 0, r;
        mTrackedArmor.id = armor.id;
        mTrackedArmor.armorPoints.assign(armor.armorPoints.begin(), armor.armorPoints.end());

        int armorId = static_cast<int>(armor.id);
        mTrackedArmor.armorType = armor.type;
        if(armor.type == ArmorType::Large && armorId >= 3 && armorId <= 5) {
            mTrackedArmor.armorNum = 2;
        } else if(armor.id == RobotType::Outpost) {
            mTrackedArmor.armorNum = 3;
        } else {
            mTrackedArmor.armorNum = 4;
        }

        mEKF.setState(mTrackedArmor.state);
        mTrackedArmor.trackingState = TrackingState::DETECTING;

        logInfo(fmt::format("CarPredictor Init EKF, target: {}, armor pos ({:.3f}, {:.3f}, {:.3f} yaw {:.3f})",
                            magic_enum::enum_name(mTrackedArmor.id), p.x, p.y, p.z, yaw));
        HubLogger::visualLog(fmt::format("CarPredictor Init EKF, target: {}, armor pos ({:.3f}, {:.3f}, {:.3f} yaw {:.3f})",
                                         magic_enum::enum_name(mTrackedArmor.id), p.x, p.y, p.z, yaw));
    }

    bool update(const double dt, const std::vector<DetectedTarget>& armors) {
        mDt = dt;
        HubLogger::watch("dt", dt);

        if(mTrackedArmor.id == RobotType::Outpost) {
            auto filterState = mEKF.getState();
            filterState(4) = 0;
            filterState(5) = 0;
            filterState(6) = 0;
            if(filterState(7) < -1.5) {
                filterState(7) = -2.512;
            } else if(filterState(7) > 1.5) {
                filterState(7) = 2.512;
            }
            filterState(8) = 0.265;

            mEKF.setState(filterState);
        }

        Eigen::VectorXd ekfPrediction;
        ekfPrediction = mEKF.predict();

        bool matched = false;
        // Use KF prediction as default target state if no matched armor is found
        mTrackedArmor.state = ekfPrediction;
        if(!armors.empty()) {
            // pair[pos,yaw]
            bool isInitCand = false;
            std::pair<glm::dvec3, double> candidate{};
            auto predictedPosition = getArmorPosFromState(ekfPrediction);
            // Difference of the current armor position and tracked armor's predicted position
            double minPositionDiff = 1000.0;

            DetectedTarget candArmor;
            for(const auto& armor : armors) {
                if(std::isnan(armor.center.mVal.x) || std::isnan(armor.center.mVal.y) || std::isnan(armor.center.mVal.z)) {
                    continue;
                }
                if(armor.id != mTrackedArmor.id) {
                    continue;
                }
                auto p = getArmorPos(armor);
                if(auto positionDiff = glm::distance(predictedPosition, p); positionDiff < minPositionDiff) {
                    minPositionDiff = positionDiff;
                    candArmor = armor;
                    candidate = { p, getArmorYaw(armor) };
                    isInitCand = true;
                    mTrackedArmor.armorPoints.assign(armor.armorPoints.begin(), armor.armorPoints.end());
                }
            }
            if(!isInitCand) {
                return false;
            }

            candidate.second = fixArmorYaw(candArmor, candidate.second);

            if(minPositionDiff < mConfig.maxMatchDist) {
                // Matching armor found
                matched = true;
                // Update EKF
                setArmorYaw(candidate.second);
                Eigen::Vector4d z(candidate.first.x, candidate.first.y, candidate.first.z, mTrackedArmor.yaw);
                mTrackedArmor.state = mEKF.update(z);
                HubLogger::visualLog(fmt::format("ArmorPredictor: EKF update Matched, minPositionDiff {:.3f}", minPositionDiff));
            } else {
                // Check if there is same id armor in current frame
                HubLogger::visualLog(fmt::format("ArmorPred check if have another same armorictor: EKF update did not matched, "
                                                 "minPositionDiff {:.3f}",
                                                 minPositionDiff));
                logInfo(fmt::format("ArmorPredictor: EKF update did not matched, minPositionDiff {:.3f}"
                                    ", check if have another same armor",
                                    minPositionDiff));
            }

            auto trackedArmorArea = cv::contourArea(mTrackedArmor.armorPoints);
            for(const auto& armor : armors) {
                auto armorArea = cv::contourArea(armor.armorPoints);
                if(armor.id == mTrackedArmor.id && trackedArmorArea < armorArea) {
                    // Armor jump may happen
                    matched = handleArmorJump(getArmorPos(armor), fixArmorYaw(armor, getArmorYaw(armor)));
                    candArmor = armor;
                    break;
                }
            }
            if(mConfig.debugView) {
                auto debugPoints = reproject(candArmor, getArmorYaw(candArmor));
                debugView(debugPoints.value());
            }

            HubLogger::watch("xRefRobot", mTrackedArmor.state(0));
            HubLogger::watch("yRefRobot", mTrackedArmor.state(1));
            HubLogger::watch("zRefRobot", mTrackedArmor.state(2));
            HubLogger::watch("yawRefRobot", glm::degrees(mTrackedArmor.state(3)));
            HubLogger::watch("R", mTrackedArmor.state(8));
            HubLogger::watch("xDetected", candidate.first.x);
            HubLogger::watch("yDetected", candidate.first.y);
            HubLogger::watch("zDetected", candidate.first.z);
            HubLogger::watch("yawDetected", glm::degrees(mTrackedArmor.yaw));
        }
        return matched;
    }

public:
    CarPredictor(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, std::move(name) }, mKey{ generateKey(this) }, mTrackedArmor{
              TimePoint(), Eigen::VectorXd::Zero(9), 0, RobotType::Negative, TrackingState::LOST
          } {
        // EKF
        // xa = x_armor, xc = x_robot_center
        // state: xc, yc, zc, yaw, v_xc, v_yc, v_zc, v_yaw, r
        // measurement: xa, ya, za, yaw
        // f - Process function
        auto f = [this](const Eigen::VectorXd& x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(4) * mDt;
            x_new(1) += x(5) * mDt;
            x_new(2) += x(6) * mDt;
            x_new(3) += x(7) * mDt;
            return x_new;
        };
        // J_f - Jacobian of process function
        auto JF = [this](const Eigen::VectorXd&) {
            Eigen::MatrixXd f(9, 9);
            // clang-format off
            f <<  1,   0,   0,   0,   mDt, 0,   0,   0,   0,
                  0,   1,   0,   0,   0,   mDt, 0,   0,   0,
                  0,   0,   1,   0,   0,   0,   mDt, 0,   0, 
                  0,   0,   0,   1,   0,   0,   0,   mDt, 0,
                  0,   0,   0,   0,   1,   0,   0,   0,   0,
                  0,   0,   0,   0,   0,   1,   0,   0,   0,
                  0,   0,   0,   0,   0,   0,   1,   0,   0,
                  0,   0,   0,   0,   0,   0,   0,   1,   0,
                  0,   0,   0,   0,   0,   0,   0,   0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto h = [](const Eigen::VectorXd& x) {
            Eigen::VectorXd z(4);
            double xc = x(0), zc = x(2), yaw = x(3), r = x(8);
            z(0) = xc + r * cos(yaw);  // xa
            z(1) = x(1);               // ya
            z(2) = zc + r * sin(yaw);  // za
            z(3) = x(3);               // yaw
            return z;
        };
        // J_h - Jacobian of observation function
        auto JH = [](const Eigen::VectorXd& x) {
            Eigen::MatrixXd h(4, 9);
            double yaw = x(3), r = x(8);
            // clang-format off
            //    xc   yc   zc   yaw         vxc  vyc  vzc  vyaw r
            h <<  1,   0,   0,   -r*sin(yaw),0,   0,   0,   0,   cos(yaw),
                  0,   1,   0,   0,          0,   0,   0,   0,   0,
                  0,   0,   1,   r*cos(yaw) ,0,   0,   0,   0,   sin(yaw),
                  0,   0,   0,   1,          0,   0,   0,   0,   0;
            // clang-format on
            return h;
        };
        // Q - process noise covariance matrix
        auto UQ = [this]() {
            Eigen::MatrixXd q(9, 9);
            double t = mDt, x = mConfig.sigma2Qxyz, y = mConfig.sigma2Qyaw, r = mConfig.sigma2QR;
            double Qxx = pow(t, 4) / 4 * x, QxVx = pow(t, 3) / 2 * x, QVxVx = pow(t, 2) * x;
            double Qyy = pow(t, 4) / 4 * y, QyVy = pow(t, 3) / 2 * x, QVyVy = pow(t, 2) * y;
            double QR = pow(t, 4) / 4 * r;

            // clang-format off
            //    xc        yc      zc      yaw     vxc     vyc     vzc     vyaw    r
            q <<    Qxx,    0,      0,      0,      QxVx,   0,      0,      0,      0,
                    0,      Qxx,    0,      0,      0,      QxVx,   0,      0,      0,
                    0,      0,      Qxx,    0,      0,      0,      QxVx,   0,      0,
                    0,      0,      0,      Qyy,    0,      0,      0,      QyVy,   0,
                    QxVx,   0,      0,      0,      QVxVx,  0,      0,      0,      0,
                    0,      QxVx,   0,      0,      0,      QVxVx,  0,      0,      0,
                    0,      0,      QxVx,   0,      0,      0,      QVxVx,  0,      0,
                    0,      0,      0,      QyVy,   0,      0,      0,      QVyVy,  0,
                    0,      0,      0,      0,      0,      0,      0,      0,      QR;
            // clang-format on

            return q;
        };

        // Eigen::DiagonalMatrix<double, 9> q;
        // q.diagonal() << mConfig.Q[0], mConfig.Q[1], mConfig.Q[2], mConfig.Q[3], mConfig.Q[4], mConfig.Q[5], mConfig.Q[6],
        //     mConfig.Q[7], mConfig.Q[8];
        // R - measurement noise covariance matrix
        auto UR = [this](const Eigen::VectorXd& z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = mConfig.Rxyz;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), mConfig.Ryaw;
            return r;
        };
        // Eigen::DiagonalMatrix<double, 4> r;
        // r.diagonal() << mConfig.R[0], mConfig.R[1], mConfig.R[2], mConfig.R[3];
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 9> p0;
        p0.setIdentity();
        mEKF = ExtendedKalmanFilter{ f, h, JF, JH, UQ, UR, p0 };
    }

    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](set_target_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(set_target_atom, TypedIdentifier<SelectedTarget>);
                ACTOR_EXCEPTION_PROBE();
                auto data = BlackBoard::instance().get<SelectedTarget>(key);

                PredictedTarget res;
                res.lastUpdate = data->lastUpdate;
                res.frame = mFrame;
                res.tfRobot2Camera = data->tfRobot2Camera;
                mTfCamera2Robot = data->tfRobot2Camera.invTransformObj();

                if(mConfig.enablePredictor) {  // 如果使用预测功能的话，目标相对机器人的速度即为机器人坐标系下，相机所观测的速度
                    if(mTrackedArmor.trackingState == TrackingState::LOST) {
                        // init
                        if(!data->selected.has_value()) {
                            return;
                        }
                        init(data->selected.value());
                    } else {
                        // update
                        if(data->lastUpdate.time_since_epoch().count() < mTrackedArmor.lastUpdate.time_since_epoch().count()) {
                            HubLogger::visualLog("CarPredictor: received pakage order wrong!");
                        }

                        bool matched = update(durationCastDouble(data->lastUpdate - mTrackedArmor.lastUpdate), data->targets);

                        // Prevent radius from spreading
                        if(mTrackedArmor.state(8) < 0.2) {
                            mTrackedArmor.state(8) = 0.2;
                            mEKF.setState(mTrackedArmor.state);
                        } else if(mTrackedArmor.state(8) > 0.4) {
                            mTrackedArmor.state(8) = 0.4;
                            mEKF.setState(mTrackedArmor.state);
                        }

                        // Tracking state machine
                        handleTrackingState(matched);

                        if(mTrackedArmor.trackingState == TrackingState::TRACKING ||
                           mTrackedArmor.trackingState == TrackingState::TEMP_LOST) {
                            res.center = glm::dvec3{ mTrackedArmor.state(0), mTrackedArmor.state(1), mTrackedArmor.state(2) };
                            res.yaw = mTrackedArmor.state(3);
                            res.linearVel = glm::dvec3{ mTrackedArmor.state(4), mTrackedArmor.state(5), mTrackedArmor.state(6) };
                            res.angularVel = mTrackedArmor.state(7);
                            res.radius = { mTrackedArmor.state(8), mLastR };
                            res.y = { mTrackedArmor.state(1), mLastY };
                            res.armorNum = mTrackedArmor.armorNum;
                            res.robotType = mTrackedArmor.id;
                            res.armorType = mTrackedArmor.armorType;
                            sendAll(car_predict_atom_v,
                                    BlackBoard::instance().updateSync<PredictedTarget>(Identifier{ mKey.val }, res));
                        }
                    }
                    mTrackedArmor.lastUpdate = data->lastUpdate;
                    HubLogger::watch("linearVelX", res.linearVel.mVal.x);
                    HubLogger::watch("linearVelY", res.linearVel.mVal.y);
                    HubLogger::watch("linearVelZ", res.linearVel.mVal.z);
                    HubLogger::watch("angularVel", res.angularVel.mVal);
                    HubLogger::visualLog(fmt::format("ArmorPredictor: Predictor Armor state: pose ({:.3f} {:.3f} {:.3f} {:.3f}), "
                                                     " linearVel ({:.3f} {:.3f} {:.3f}) angularVel {:.3f}, r {:.3f}",
                                                     mTrackedArmor.state(0), mTrackedArmor.state(1), mTrackedArmor.state(2),
                                                     mTrackedArmor.state(3), mTrackedArmor.state(4), mTrackedArmor.state(5),
                                                     mTrackedArmor.state(6), mTrackedArmor.state(7), mTrackedArmor.state(8)));
                } else {  // 如果不使用预测功能的话，将目标看作为静止状态，目标相对机器人的速度即为机器人自身速度取反
                    const auto dataPosture = BlackBoard::instance().get<PostureData>(mIMUKey);
                    // logInfo("ArmorPredictor receive");
                    HubLogger::visualLog("ArmorPredictor receive");
                    if(!dataPosture.has_value() || !data->selected.has_value()) {
                        return;
                    }
                    res.center = getArmorPos(data->selected.value());
                    res.yaw = getArmorYaw(data->selected.value());
                    res.linearVel = -dataPosture->linearVelocityOfRobot.mVal;
                    res.angularVel = 0;
                    res.radius = { 0, 0 };
                    res.y = { res.center.mVal.y, res.center.mVal.y };
                    res.armorNum = 1;
                    res.robotType = data->selected.value().id;
                    res.armorType = data->selected.value().type;
                    HubLogger::visualLog(fmt::format("ArmorPredictor do not use predict func, position : ({:.3f} {:.3f} {:.3f}), "
                                                     "linearVel: ({:.3f} {:.3f} {:.3f})",
                                                     res.center.mVal.x, res.center.mVal.y, res.center.mVal.z,
                                                     res.linearVel.mVal.x, res.linearVel.mVal.y, res.linearVel.mVal.z));
                    sendAll(car_predict_atom_v, BlackBoard::instance().updateSync<PredictedTarget>(Identifier{ mKey.val }, res));
                }
            },
            [this](update_posture_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(update_posture_atom, TypedIdentifier<PostureData>);
                mIMUKey = key;
            },
            [this](image_frame_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame, std::string_view>);
                if(frameInit) {
                    return;
                }
                const auto data = BlackBoard::instance().get<CameraFrame, std::string_view>(key).value();
                auto frame = std::get<0>(data);
                mFrame = frame;
                frameInit = true;
            }
        };
    }
};

HUB_REGISTER_CLASS(CarPredictor);