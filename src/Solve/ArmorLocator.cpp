#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedTarget.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <glm/glm.hpp>
#include <opencv2/calib3d.hpp>

struct ArmorLocatorSettings final {};

template <class Inspector>
bool inspect(Inspector& f, ArmorLocatorSettings& x) {
    return f.object(x).fields();
}

class ArmorLocator final : public HubHelper<caf::event_based_actor, ArmorLocatorSettings, detect_available_atom> {
    Identifier mKey, mHeadKey{};
    std::vector<cv::Point3f> mObjectPointsR1, mObjectPointsR2;
    std::vector<cv::Point2f> mImagePoint{ 4 };

    void initializePoints() {
        mObjectPointsR1 = {
            cv::Point3f(-widthOfSmallArmor / 2, -heightOfArmorLightBar / 2, 0),
            cv::Point3f(-widthOfSmallArmor / 2, +heightOfArmorLightBar / 2, 0),
            cv::Point3f(-(widthOfSmallArmor / 2 - heightOfArmorLightBar), +heightOfArmorLightBar / 2, 0),
            cv::Point3f(-(widthOfSmallArmor / 2 - heightOfArmorLightBar), -heightOfArmorLightBar / 2, 0),
        };
        mObjectPointsR2 = {
            cv::Point3f(+(widthOfSmallArmor / 2 - heightOfArmorLightBar), -heightOfArmorLightBar / 2, 0),
            cv::Point3f(+(widthOfSmallArmor / 2 - heightOfArmorLightBar), +heightOfArmorLightBar / 2, 0),
            cv::Point3f(+widthOfSmallArmor / 2, +heightOfArmorLightBar / 2, 0),
            cv::Point3f(+widthOfSmallArmor / 2, -heightOfArmorLightBar / 2, 0),
        };
    }

    Point<UnitType::Distance, FrameOfReference::Camera> solve(const cv::Mat& cameraMatrix, const PairedLight& armor) {
        cv::Mat_<double> distCoeff;
        cv::Mat revc, tvec;

        armor.r1.points(mImagePoint.data());
        const auto res1 =
            cv::solvePnP(mObjectPointsR1, mImagePoint, cameraMatrix, distCoeff, revc, tvec, false, cv::SOLVEPNP_IPPE);
        const glm::dvec3 p0 = { tvec.at<double>(0, 0), -tvec.at<double>(1, 0), -tvec.at<double>(2, 0) };

        armor.r2.points(mImagePoint.data());
        const auto res2 =
            cv::solvePnP(mObjectPointsR2, mImagePoint, cameraMatrix, distCoeff, revc, tvec, false, cv::SOLVEPNP_IPPE);
        const glm::dvec3 p1 = { tvec.at<double>(0, 0), -tvec.at<double>(1, 0), -tvec.at<double>(2, 0) };

        if(res1 && res2)
            return Point<UnitType::Distance, FrameOfReference::Camera>(0.5 * (p0 + p1));
        if(res1)
            return Point<UnitType::Distance, FrameOfReference::Camera>(p0);
        return Point<UnitType::Distance, FrameOfReference::Camera>(p1);
    }

public:
    ArmorLocator(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(ArmorLocator).hash_code() } {
        initializePoints();
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](armor_detect_available_atom, Identifier key) {
                     const auto data = BlackBoard::instance().get<DetectedArmorArray>(key).value();
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

                     const cv::Mat cameraMatrix =
                         (cv::Mat_<double>(3, 3) << cameraInfo.width / 2 / tan(glm::radians(cameraInfo.fov) / 2), 0,
                          cameraInfo.width / 2, 0, cameraInfo.height / 2 / tan(glm::radians(cameraInfo.fov) / 2),
                          cameraInfo.height / 2, 0, 0, 1);

                     for(auto& cars : data.armors) {
                         for(auto& armor : cars.armors) {
                             auto armorLight = armor;
                             armorLight.r1.center += cv::Point2f{ cars.roi.tl() };
                             armorLight.r2.center += cv::Point2f{ cars.roi.tl() };

                             const auto point = solve(cameraMatrix, armorLight);

                             // TODO: projected area
                             res.targets.push_back({ transform(point), 0.0, cars.id });
                         }
                     }

                     BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(detect_available_atom_v, mKey);
                 },
                 [&](update_head_atom, Identifier key) { mHeadKey = key; } };
    }
};

HUB_REGISTER_CLASS(ArmorLocator);
