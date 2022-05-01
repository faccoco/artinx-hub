#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedTarget.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <fmt/format.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <opencv2/calib3d.hpp>

struct ArmorLocatorSettings final {};

template <class Inspector>
bool inspect(Inspector& f, ArmorLocatorSettings& x) {
    return f.object(x).fields();
}

class ArmorLocator final : public HubHelper<caf::event_based_actor, ArmorLocatorSettings, detect_available_atom> {
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

    Point<UnitType::Distance, FrameOfReference::Camera> solve(const cv::Mat& cameraMatrix, const PairedLight& armor) {
        boxRect(mImagePoint, armor.r1);

        const cv::Point2d lt = 0.5 * (mImagePoint[1] + mImagePoint[2]);
        const cv::Point2d lb = 0.5 * (mImagePoint[0] + mImagePoint[3]);

        boxRect(mImagePoint, armor.r2);

        const cv::Point2d rt = 0.5 * (mImagePoint[1] + mImagePoint[2]);
        const cv::Point2d rb = 0.5 * (mImagePoint[0] + mImagePoint[3]);

        const cv::Mat_<double> distCoeff;
        cv::Mat rvec, tvec;

        const auto left = 0.5 * (lt + lb);
        const auto right = 0.5 * (rt + rb);

        const auto distHorizontal = std::hypot(left.x - right.x, left.y - right.y);
        const auto distVertical = 0.5 * (std::hypot(lb.x - lt.x, lb.y - lt.y) + std::hypot(rb.x - rt.x, rb.y - rt.y));

        const auto ratio = distHorizontal / distVertical;
        constexpr auto ratioThreshold = 0.5 * (widthOfLargeArmor + widthOfSmallArmor) / heightOfArmorLightBar;

        // std::cout << (ratio > ratioThreshold ? "large" : "small") << std::endl;

        mImagePoint = { lt, lb, rb, rt };
        const auto res = cv::solvePnP(ratio > ratioThreshold ? mObjectPointsLarge : mObjectPointsSmall, mImagePoint, cameraMatrix,
                                      distCoeff, rvec, tvec, false, cv::SOLVEPNP_IPPE);
        glm::dvec3 p0 = { tvec.at<double>(0, 0), -tvec.at<double>(1, 0), -tvec.at<double>(2, 0) };

        if(p0.z > 0.0)
            p0 = -p0;

        return Point<UnitType::Distance, FrameOfReference::Camera>{ p0 };
    }

public:
    ArmorLocator(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(ArmorLocator).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](armor_detect_available_atom, Identifier key) {
                     const auto data = BlackBoard::instance().get<DetectedArmorArray>(key).value();
                     //                     logInfo(data.armors[0].armors.size());
                     DetectedTargetArray res;
                     res.lastUpdate = data.frame.lastUpdate;
                     const auto& cameraInfo = data.frame.info;

                     Transform<FrameOfReference::Gun, FrameOfReference::Camera, true> transform;
                     if(cameraInfo.transform.index() == 0) {
                         transform = std::get<0>(cameraInfo.transform);
                     } else {
                         const auto& trans = std::get<1>(cameraInfo.transform);
                         const auto headTrans = BlackBoard::instance().get<HeadInfo>(mHeadKey).value().transform;
                         transform =
                             static_cast<Transform<FrameOfReference::Gun, FrameOfReference::Robot, true>>(headTrans) * trans;
                     }

                     /*
                     transform = Transform<FrameOfReference::Gun, FrameOfReference::Camera, true>{ glm::translate(
                         glm::identity<glm::dmat4>(), glm::dvec3{ 0.0, 0.1, -0.15 }) };
                         */

                     const cv::Mat cameraMatrix =
                         (cv::Mat_<double>(3, 3) << cameraInfo.width / 2 / tan(glm::radians(cameraInfo.fov) / 2), 0,
                          cameraInfo.width / 2, 0, cameraInfo.height / 2 / tan(glm::radians(cameraInfo.fov) / 2),
                          cameraInfo.height / 2, 0, 0, 1);

                     for(const auto& [roi, id, armors] : data.armors) {
                         for(auto& armor : armors) {
                             auto armorLight = armor;
                             armorLight.r1.center += cv::Point2f{ roi.tl() };
                             armorLight.r2.center += cv::Point2f{ roi.tl() };

                             const auto point = solve(cameraMatrix, armorLight);
                             std::cout << distance(point, {}).val << std::endl;

                             // TODO: projected area
                             res.targets.push_back({ transform(point), 0.0, id });
                         }
                     }

                     if(!res.targets.empty()) {
                         auto center = res.targets[0].center.raw();
                         //                         std::cout << fmt::format("x: {} y: {} z: {}", center.x, center.y, center.z) <<
                         //                         std::endl;
                     }

                     BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(detect_available_atom_v, mKey);
                 },
                 [&](update_head_atom, Identifier key) { mHeadKey = key; } };
    }
};

HUB_REGISTER_CLASS(ArmorLocator);
