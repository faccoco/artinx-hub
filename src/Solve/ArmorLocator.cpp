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

struct ArmorLocatorSettings final {
    std::vector<int> largeArmor;
};

template <class Inspector>
bool inspect(Inspector& f, ArmorLocatorSettings& x) {
    return f.object(x).fields(f.field("largeArmor", x.largeArmor).fallback(std::vector<int>()));
}

class ArmorLocator final
    : public HubHelper<caf::event_based_actor, ArmorLocatorSettings, detect_available_atom, image_frame_atom> {
    Identifier mKey /*, mHeadKey{}*/;
    std::set<int> mLargeArmor;

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

    std::pair<Point<UnitType::Distance, FrameOfRef::Camera>, Vector<UnitType::Distance, FrameOfRef::Camera>>
    solve([[maybe_unused]] cv::Mat& debugView, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, bool isLargeArmor) {
        cv::Mat rvec, tvec;

        [[maybe_unused]] const auto res = cv::solvePnP(isLargeArmor ? mObjectPointsLarge : mObjectPointsSmall, mImagePoint,
                                                       cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);
        glm::dvec3 p0 = { tvec.at<double>(0, 0), -tvec.at<double>(1, 0), -tvec.at<double>(2, 0) };
        glm::dvec3 r = { rvec.at<double>(0, 0), -rvec.at<double>(1, 0), -rvec.at<double>(2, 0) };

#ifdef ARTINXHUB_DEBUG
        cv::drawFrameAxes(debugView, cameraMatrix, distCoeffs, rvec, tvec,
                          static_cast<float>(isLargeArmor ? widthOfLargeArmor : widthOfSmallArmor) * 0.5f);
#endif

        return std::make_pair(Point<UnitType::Distance, FrameOfRef::Camera>{ p0 },
                              Vector<UnitType::Distance, FrameOfRef::Camera>{ r });
    }

public:
    ArmorLocator(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        for(auto num : mConfig.largeArmor) {
            mLargeArmor.insert(num);
        }
    }
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
                res.tfRobot2Gun = cameraInfo.tfRobot2Gun;

                auto debugView = data.frame.frame.clone();

                auto tfCamera2Gun = data.frame.info.tfGun2Camera.invTransformObj();

                cv::Point2f imgCenter{ data.frame.frame.size[0] / 2.f, data.frame.frame.size[1] / 2.f };
                for(const auto& armor : data.armors) {
                    mImagePoint = armor.light4Point;

                    bool isLargeArmor = mLargeArmor.count(static_cast<int>(armor.robotType)) > 0;
                    auto armorType = isLargeArmor ? ArmorType::Large : ArmorType::Small;
                    auto [point, rvec] = solve(debugView, cameraInfo.cameraMatrix, cameraInfo.distCoefficients, isLargeArmor);

                    auto pointRefGun = tfCamera2Gun(point);
                    Transform<FrameOfRef::Armor, FrameOfRef::Camera> rmat;

                    double angle = glm::length(rvec.mVal);
                    auto axis = rvec.mVal / angle;
                    rmat = glm::mat4_cast(glm::angleAxis(-angle, axis));

                    HubLogger::watch("XRefCam", point.mVal.x);
                    HubLogger::watch("YRefCam", point.mVal.y);
                    HubLogger::watch("ZRefCam", point.mVal.z);
                    // HubLogger::watch("YawRefCam", glm::degrees(-atan2(rmat.raw()[2][0], rmat.raw()[2][2])));

                    auto rmatRefGun = combine(tfCamera2Gun, rmat);
                    auto armorImgCenter = clcArmorImgCenter();
                    res.targets.push_back({ armorImgCenter, distance2D(armorImgCenter, imgCenter), pointRefGun, armor.robotType,
                                            armorType, ArmorMotion::Unsure, rmatRefGun });
                    HubLogger::VisualLog(fmt::format("ArmorLocator locate target: RobotType:{}, ArmorImgCenter:({:.2f}, "
                                                     "{:.2f}), PositionRefGun:({:.2f}, {:.2f}, {:.2f})",
                                                     magic_enum::enum_name(armor.robotType), armorImgCenter.x, armorImgCenter.y,
                                                     pointRefGun.mVal.x, pointRefGun.mVal.y, pointRefGun.mVal.z));
                    //                     logInfo(fmt::format("Position ref Camera: x:{:.3}, y:{:.3}, z:{:.3} ArmorType:{}",
                    //                     pointRefGun.mVal.x,
                    //                                        pointRefGun.mVal.y, pointRefGun.mVal.z, isLargeArmor));
                }

                sendAll(detect_available_atom_v, mGroupMask, BlackBoard::instance().updateSync(mKey, std::move(res)));
            }

        };
    }
};

HUB_REGISTER_CLASS(ArmorLocator);
