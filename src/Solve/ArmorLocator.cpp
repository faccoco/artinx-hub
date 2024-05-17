#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedTarget.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <opencv2/calib3d.hpp>

struct ArmorLocatorSettings final {};

template <class Inspector>
bool inspect(Inspector& f, ArmorLocatorSettings& x) {
    return f.object(x).fields();
}

class ArmorLocator final
    : public HubHelper<caf::event_based_actor, ArmorLocatorSettings, detect_available_atom, image_frame_atom> {
    Identifier mKey /*, mHeadKey{}*/;

    const std::vector<cv::Point3d> mObjectPointsSmall = {
        { -widthOfSmallArmor / 2, +heightOfArmorLightBar / 2, 0.0 },
        { -widthOfSmallArmor / 2, -heightOfArmorLightBar / 2, 0.0 },
        { +widthOfSmallArmor / 2, -heightOfArmorLightBar / 2, 0.0 },
        { +widthOfSmallArmor / 2, +heightOfArmorLightBar / 2, 0.0 },
    };
    const std::vector<cv::Point3d> mObjectPointsLarge = {
        { -widthOfLargeArmor / 2, +heightOfArmorLightBar / 2, 0.0 },
        { -widthOfLargeArmor / 2, -heightOfArmorLightBar / 2, 0.0 },
        { +widthOfLargeArmor / 2, -heightOfArmorLightBar / 2, 0.0 },
        { +widthOfLargeArmor / 2, +heightOfArmorLightBar / 2, 0.0 },
    };
    std::vector<cv::Point2f> mImagePoint{ 4 };

    cv::Point2f clcArmorImgCenter() {
        return { (mImagePoint[0].x + mImagePoint[1].x + mImagePoint[2].x + mImagePoint[3].x) / 4,
                 (mImagePoint[0].y + mImagePoint[1].y + mImagePoint[2].y + mImagePoint[3].y) / 4 };
    }

public:
    ArmorLocator(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, std::move(name) }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [&](armor_detect_available_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(armor_detect_available_atom, TypedIdentifier<DetectedArmorArray>);
                ACTOR_EXCEPTION_PROBE();

                auto data = BlackBoard::instance().get<DetectedArmorArray>(key).value();
                DetectedTargetArray res;
                res.lastUpdate = data.frame.lastUpdate;
                const auto& cameraInfo = data.frame.info;
                res.tfRobot2Camera = cameraInfo.tfRobot2Camera;

                cv::Point2f imgCenter{ data.frame.frame.cols / 2.f, data.frame.frame.rows / 2.f };
                for(const auto& armor : data.armors) {
                    mImagePoint = armor.light4Point;
                    auto armorType = armor.isLargeArmor ? ArmorType::Large : ArmorType::Small;

                    cv::Mat rvec, tvec;
                    const auto pnpRes =
                        cv::solvePnP(armor.isLargeArmor ? mObjectPointsLarge : mObjectPointsSmall, mImagePoint,
                                     cameraInfo.cameraMatrix, cameraInfo.distCoefficients, rvec, tvec, false, cv::SOLVEPNP_IPPE);
                    if(!pnpRes) {
                        continue;
                    }
                    glm::dvec3 p0 = { tvec.at<double>(0, 0), -tvec.at<double>(1, 0), -tvec.at<double>(2, 0) };
                    glm::dvec3 r = { rvec.at<double>(0, 0), -rvec.at<double>(1, 0), -rvec.at<double>(2, 0) };

                    auto pointRefCam = Point<UnitType::Distance, FrameOfRef::Camera>{ p0 };
                    auto rvecRefCam = Vector<UnitType::Distance, FrameOfRef::Camera>{ r };

                    double angle = glm::length(rvecRefCam.mVal);
                    auto axis = rvecRefCam.mVal / angle;
                    auto rmat = glm::mat4_cast(glm::angleAxis(-angle, axis));

                    HubLogger::watch("XRefCam", p0.x);
                    HubLogger::watch("YRefCam", p0.y);
                    HubLogger::watch("ZRefCam", p0.z);
                    // HubLogger::watch("isLargeArmor", armor.isLargeArmor);
                    // logInfo(fmt::format("isLargeArmor: {} {}", armor.ratio, isLargeArmor));
                    HubLogger::watch("YawRefCam", glm::degrees(normalizeAngle(-atan2(rmat[2][0], rmat[2][2]) -
                    glm::half_pi<double>())));

                    auto armorImgCenter = clcArmorImgCenter();
                    res.targets.push_back({ armorImgCenter, distance2D(armorImgCenter, imgCenter), pointRefCam, armor.robotType,
                                            armorType, ArmorMotion::Unsure,
                                            Transform<FrameOfRef::Armor, FrameOfRef::Camera>(rmat) });

                    if(std::isnan(p0.x) || std::isnan(p0.y) || std::isnan(p0.z)) {
                        HubLogger::visualLog("ArmorLocator: nan orrcur");
                    }

                    HubLogger::visualLog(
                        fmt::format("ArmorLocator locate target: RobotType:{}, ArmorImgCenter:({:.2f}, "
                                    "{:.2f}), Position:({:.2f}, {:.2f}, {:.2f}), Yaw:{:.2f}",
                                    magic_enum::enum_name(armor.robotType), armorImgCenter.x, armorImgCenter.y, p0.x, p0.y, p0.z,
                                    glm::degrees(normalizeAngle(-atan2(rmat[2][0], rmat[2][2]) - glm::half_pi<double>()))));
                }

                sendAll(detect_available_atom_v, mGroupMask, BlackBoard::instance().updateSync(mKey, std::move(res)));
            },
        };
    }
};

HUB_REGISTER_CLASS(ArmorLocator);
