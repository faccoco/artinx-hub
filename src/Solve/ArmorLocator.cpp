#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedTarget.hpp"
#include "ExceptionProbe.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <opencv2/calib3d.hpp>

#include "SuppressWarningEnd.hpp"

struct ArmorLocatorSettings final {
    float ratioThreshold;
    double ky2kz2x, my2kz2x, ky2mz2x, my2mz2x;
    double kz2y, mz2y;
    double kz2z, mz2z;
};

template <class Inspector>
bool inspect(Inspector& f, ArmorLocatorSettings& x) {
    return f.object(x).fields(f.field("ratioThreshold", x.ratioThreshold), f.field("ky2kz2x", x.ky2kz2x).fallback(0.f),
                              f.field("my2kz2x", x.my2kz2x).fallback(0.f), f.field("ky2mz2x", x.ky2mz2x).fallback(0.f),
                              f.field("my2mz2x", x.my2mz2x).fallback(0.f), f.field("kz2y", x.kz2y).fallback(0.f),
                              f.field("mz2y", x.mz2y).fallback(0.f), f.field("kz2z", x.kz2z).fallback(0.f),
                              f.field("mz2z", x.mz2z).fallback(0.f));
}

class ArmorLocator final
    : public HubHelper<caf::event_based_actor, ArmorLocatorSettings, detect_available_atom, image_frame_atom> {
    Identifier mKey, mHeadKey{};
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

    static double evalArea(cv::Point2d p1, cv::Point2d p2, cv::Point2d p3) {
        const auto v1 = p2 - p1;
        const auto v2 = p3 - p1;
        return v1.x * v2.y - v1.y * v2.x;
    }

    static double evalArea(cv::Point2d p1, cv::Point2d p2, cv::Point2d p3, cv::Point2d p4) {
        return -evalArea(p1, p2, p3) - evalArea(p1, p3, p4);
    }

    cv::Point2f clcArmorImgCenter() {
        return { (mImagePoint[0].x + mImagePoint[1].x + mImagePoint[2].x + mImagePoint[3].x) / 4,
                 (mImagePoint[0].y + mImagePoint[1].y + mImagePoint[2].y + mImagePoint[3].y) / 4 };
    }

    bool initImgPointAndArmorType(const PairedLight& armor) {
        boxRect(mImagePoint, armor.r1);
        const auto area1 = armor.r1.size.area();

        const cv::Point2d lt = 0.5 * (mImagePoint[1] + mImagePoint[2]);
        const cv::Point2d lb = 0.5 * (mImagePoint[0] + mImagePoint[3]);

        boxRect(mImagePoint, armor.r2);
        const auto area2 = armor.r2.size.area();

        const cv::Point2d rt = 0.5 * (mImagePoint[1] + mImagePoint[2]);
        const cv::Point2d rb = 0.5 * (mImagePoint[0] + mImagePoint[3]);

        mImagePoint = { lt, lb, rb, rt };
        const auto area = evalArea(lt, lb, rb, rt);

        const auto ratio = area / std::fmax(0.001, area1 + area2);
        HubLogger::watch("armor ratio", ratio);

        return ratio > mConfig.ratioThreshold;
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
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](armor_detect_available_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(armor_detect_available_atom, TypedIdentifier<DetectedArmorArray>);
                     ACTOR_EXCEPTION_PROBE();

                     auto data = BlackBoard::instance().get<DetectedArmorArray>(key).value();
                     DetectedTargetArray res;
                     res.lastUpdate = data.frame.lastUpdate;
                     res.tfRobot2Gun = data.frame.info.tfRobot2Gun;
                     const auto& cameraInfo = data.frame.info;

                     auto debugView = data.frame.frame.clone();

                     auto tfCamera2Gun = cameraInfo.tfGun2Camera.invTransformObj();

                     for(const auto& [id, armor] : data.armors) {

                         bool isLargeArmor = initImgPointAndArmorType(armor);

                         const auto point = solve(debugView, cameraInfo.cameraMatrix, cameraInfo.distCoefficients, isLargeArmor);

                         res.targets.push_back({ clcArmorImgCenter(), tfCamera2Gun(point), 0.0, id,
                                                 isLargeArmor ? ArmorType::Large : ArmorType::Small });
                         //  logInfo(fmt::format("Armor Type:{}, Position ref Gun: x:{}, y:{} z:{}", isLargeArmor, point.mVal.x,
                         //                      point.mVal.y, point.mVal.z));
                     }

#ifdef ARTINXHUB_DEBUG
                     std::swap(debugView, data.frame.frame);
                     sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(mKey, std::move(data.frame)));
#endif

                     sendAll(detect_available_atom_v, mGroupMask, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 },
                 [&](armor_nnet_detect_available_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(armor_nnet_detect_available_atom, TypedIdentifier<NNetDetectedArmorArray>);
                     ACTOR_EXCEPTION_PROBE();

                     auto data = BlackBoard::instance().get<NNetDetectedArmorArray>(key).value();
                     DetectedTargetArray res;
                     res.lastUpdate = data.frame.lastUpdate;
                     const auto& cameraInfo = data.frame.info;
                     res.tfRobot2Gun = cameraInfo.tfRobot2Gun;

                     auto debugView = data.frame.frame.clone();

                     auto tfCamera2Gun = data.frame.info.tfGun2Camera.invTransformObj();

                     for(const auto& armor : data.armors) {
                         mImagePoint = armor.light4Point;

                         bool isLargeArmor = armor.robotType >= 2 && armor.robotType <= 6 ? false : true;
                         auto point = solve(debugView, cameraInfo.cameraMatrix, cameraInfo.distCoefficients, isLargeArmor);

                         point.mVal.z += (point.mVal.z - mConfig.mz2z) / (mConfig.kz2z + 1);
                         point.mVal.y += mConfig.kz2y * point.mVal.z + mConfig.mz2y;
                         point.mVal.x += (mConfig.ky2kz2x * point.mVal.y + mConfig.my2kz2x) * point.mVal.z +
                             (mConfig.ky2mz2x * point.mVal.y + mConfig.my2mz2x);

                         res.targets.push_back({ clcArmorImgCenter(), tfCamera2Gun(point), 0.0, armor.robotType,
                                                 isLargeArmor ? ArmorType::Large : ArmorType::Small });
                         logInfo(fmt::format("Armor Type:{}, Position ref Gun: x:{:.3}, y:{:.3} z:{:.3}", isLargeArmor,
                                             point.mVal.x, point.mVal.y, point.mVal.z));
                     }

                     sendAll(detect_available_atom_v, mGroupMask, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 }

        };
    }
};

HUB_REGISTER_CLASS(ArmorLocator);
