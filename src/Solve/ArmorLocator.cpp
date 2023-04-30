#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedTarget.hpp"
#include "ExceptionProbe.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/glm.hpp>
#include <opencv2/calib3d.hpp>

#include "SuppressWarningEnd.hpp"

struct ArmorLocatorSettings final {
    float ratioThreshold;
    std::vector<int> largeArmor, smallArmor;
};

template <class Inspector>
bool inspect(Inspector& f, ArmorLocatorSettings& x) {
    return f.object(x).fields(f.field("ratioThreshold", x.ratioThreshold),
                              f.field("largeArmor", x.largeArmor).fallback(std::vector<int>()),
                              f.field("smallArmor", x.smallArmor).fallback(std::vector<int>()));
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

    Point<UnitType::Distance, FrameOfRef::Camera> solve([[maybe_unused]] cv::Mat& debugView, const cv::Mat& cameraMatrix,
                                                        const cv::Mat& distCoeffs, bool isLargeArmor) {
        cv::Mat rvec, tvec;

        [[maybe_unused]] const auto res = cv::solvePnP(isLargeArmor ? mObjectPointsLarge : mObjectPointsSmall, mImagePoint,
                                                       cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);
        glm::dvec3 p0 = { tvec.at<double>(0, 0), -tvec.at<double>(1, 0), -tvec.at<double>(2, 0) };

        if(p0.z > 0.0)
            p0 = -p0;

#ifdef ARTINXHUB_DEBUG
        cv::drawFrameAxes(debugView, cameraMatrix, distCoeffs, rvec, tvec,
                          static_cast<float>(isLargeArmor ? widthOfLargeArmor : widthOfSmallArmor) * 0.5f);
#endif

        return Point<UnitType::Distance, FrameOfRef::Camera>{ p0 };
    }

public:
    ArmorLocator(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
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

                     for(const auto& armor : data.armors) {
                         mImagePoint = armor.light4Point;

                         bool isLargeArmor = false;
                         for(auto num : mConfig.largeArmor)
                             if(static_cast<RobotType>(num) == armor.robotType)
                                 isLargeArmor = true;
                         auto point = solve(debugView, cameraInfo.cameraMatrix, cameraInfo.distCoefficients, isLargeArmor);

                         auto pointRefGun = tfCamera2Gun(point);
                         res.targets.push_back({ clcArmorImgCenter(), pointRefGun, 0.0, armor.robotType,
                                                 isLargeArmor ? ArmorType::Large : ArmorType::Small });
                         logInfo(fmt::format("Position ref Camera: x:{:.3}, y:{:.3}, z:{:.3} Armor Type:{}", point.mVal.x,
                                             point.mVal.y, point.mVal.z, isLargeArmor));
                         //                         logInfo(fmt::format("Armor Type:{}, Position ref Gun: x:{:.3}, y:{:.3}
                         //                         z:{:.3}", isLargeArmor,
                         //                                             pointRefGun.mVal.x, pointRefGun.mVal.y,
                         //                                             pointRefGun.mVal.z));
                     }

                     sendAll(detect_available_atom_v, mGroupMask, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 }

        };
    }
};

HUB_REGISTER_CLASS(ArmorLocator);
