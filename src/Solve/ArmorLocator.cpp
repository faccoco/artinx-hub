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
#include <fmt/format.h>

struct ArmorLocatorSettings final {};

template <class Inspector>
bool inspect(Inspector& f, ArmorLocatorSettings& x) {
    return f.object(x).fields();
}

class ArmorLocator final : public HubHelper<caf::event_based_actor, ArmorLocatorSettings, detect_available_atom> {
    Identifier mKey, mHeadKey{};
    // 1 2
    // 0 3
    const std::vector<cv::Point3d> mObjectPointsR1 = {
        { -widthOfSmallArmor / 2, +heightOfArmorLightBar / 2, 0.0 },
        { -widthOfSmallArmor / 2, -heightOfArmorLightBar / 2, 0.0 },
        { -(widthOfSmallArmor / 2 - widthOfArmorLightBar), -heightOfArmorLightBar / 2, 0.0 },
        { -(widthOfSmallArmor / 2 - widthOfArmorLightBar), +heightOfArmorLightBar / 2, 0.0 },
        { 0.0, 0.0, 0.0 }
    };
    const std::vector<cv::Point3d> mObjectPointsR2 = {
        { +(widthOfSmallArmor / 2 - widthOfArmorLightBar), +heightOfArmorLightBar / 2, 0.0 },
        { +(widthOfSmallArmor / 2 - widthOfArmorLightBar), -heightOfArmorLightBar / 2, 0.0 },
        { +widthOfSmallArmor / 2, -heightOfArmorLightBar / 2, 0.0 },
        { +widthOfSmallArmor / 2, +heightOfArmorLightBar / 2, 0.0 },
        { 0.0, 0.0, 0.0 }
    };
    std::vector<cv::Point2f> mImagePoint{ 5 };

    static double cross(cv::Point2d a, cv::Point2d b) noexcept {
        return a.x * b.y - b.x * a.y;
    }

    // width < height
    // angle = 0
    // 1 width 2
    // height  height
    // 0 width 3
    // angle = 90
    // 0 height 1
    // width    width
    // 3 height 2

    void boxRect(std::vector<cv::Point2f>& res, const cv::RotatedRect& rect) {
        rect.points(res.data());

        uint32_t selectedIdx = 0;
        double minX = 1e5;

        for(uint32_t idx = 0; idx < 4; ++idx)
            if(res[idx].x < minX && res[idx].y > res[(idx + 2) % 4].y) {
                selectedIdx = idx;
                minX = res[idx].x;
            }

        if(selectedIdx)
            std::rotate(res.begin(), res.begin() + selectedIdx, res.begin() + 4);
    }

    cv::Point2f solveCenter(const PairedLight& armor) {
        boxRect(mImagePoint, armor.r1);

        const cv::Point2d lt = 0.5 * (mImagePoint[1] + mImagePoint[2]);
        const cv::Point2d lb = 0.5 * (mImagePoint[0] + mImagePoint[3]);

        boxRect(mImagePoint, armor.r2);

        const cv::Point2d rt = 0.5 * (mImagePoint[1] + mImagePoint[2]);
        const cv::Point2d rb = 0.5 * (mImagePoint[0] + mImagePoint[3]);

        const auto oriA = lb, dirA = rt - lb;
        const auto oriB = lt, dirB = rb - lt;
        const auto delta = oriA - oriB;
        const auto crossA = cross(dirB, delta);
        const auto crossB = cross(dirA, dirB);
        if(std::fabs(crossB) < 1e-5)
            return 0.5 * (armor.r1.center + armor.r2.center);
        return oriA + dirA * (crossA / crossB);
    }

    glm::dvec3 solveFallback(const cv::Mat& cameraMatrix, const PairedLight& armor) {
        const std::vector<cv::Point3d> objectPointsR1{ mObjectPointsR1.begin(), mObjectPointsR1.begin() + 4 };
        const std::vector<cv::Point3d> objectPointsR2{ mObjectPointsR2.begin(), mObjectPointsR2.begin() + 4 };

        std::vector<cv::Point2f> imagePoint{ 4 };

        cv::Mat_<double> distCoeff;
        cv::Mat revc, tvec;
        boxRect(imagePoint, armor.r1);
        const auto res1 = cv::solvePnP(objectPointsR1, imagePoint, cameraMatrix, distCoeff, revc, tvec, false, cv::SOLVEPNP_IPPE);
        const glm::dvec3 p0 = { tvec.at<double>(0, 0), -tvec.at<double>(1, 0), -tvec.at<double>(2, 0) };

        boxRect(imagePoint, armor.r2);
        const auto res2 = cv::solvePnP(objectPointsR2, imagePoint, cameraMatrix, distCoeff, revc, tvec, false, cv::SOLVEPNP_IPPE);
        const glm::dvec3 p1 = { tvec.at<double>(0, 0), -tvec.at<double>(1, 0), -tvec.at<double>(2, 0) };

        if(res1 && res2 && p0.z < 0.0 && p1.z < 0.0)
            return 0.5 * (p0 + p1);
        if(res1 && p0.z < 0.0)
            return p0;
        return p1;
    }

    Point<UnitType::Distance, FrameOfReference::Camera> solve(const cv::Mat& cameraMatrix, const PairedLight& armor) {
        const auto center = solveCenter(armor);
        mImagePoint[4] = center;

        cv::Mat_<double> distCoeff;
        cv::Mat revc, tvec;

        boxRect(mImagePoint, armor.r1);
        const auto res1 =
            cv::solvePnP(mObjectPointsR1, mImagePoint, cameraMatrix, distCoeff, revc, tvec, false, cv::SOLVEPNP_IPPE);
        const glm::dvec3 p0 = { tvec.at<double>(0, 0), -tvec.at<double>(1, 0), -tvec.at<double>(2, 0) };

        boxRect(mImagePoint, armor.r2);
        const auto res2 =
            cv::solvePnP(mObjectPointsR2, mImagePoint, cameraMatrix, distCoeff, revc, tvec, false, cv::SOLVEPNP_IPPE);
        const glm::dvec3 p1 = { tvec.at<double>(0, 0), -tvec.at<double>(1, 0), -tvec.at<double>(2, 0) };

        if(res1 && res2 && p0.z < 0.0 && p1.z < 0.0)
            return Point<UnitType::Distance, FrameOfReference::Camera>{ 0.5 * (p0 + p1) };
        if(res1 && p0.z < 0.0)
            return Point<UnitType::Distance, FrameOfReference::Camera>{ p0 };
        if(res2 && p1.z < 0.0)
            return Point<UnitType::Distance, FrameOfReference::Camera>{ p1 };
        return Point<UnitType::Distance, FrameOfReference::Camera>{ solveFallback(cameraMatrix, armor) };
    }

public:
    ArmorLocator(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(ArmorLocator).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](armor_detect_available_atom, Identifier key) {
                     const auto data = BlackBoard::instance().get<DetectedArmorArray>(key).value();
//                     CAF_LOG_INFO(data.armors[0].armors.size());
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


                             cv::Point2f pts[4];
                             std::vector<cv::Point2f> pts8;
                             pts8.reserve(8);
                             armorLight.r1.points(pts);
                             pts8.insert(pts8.cend(), pts, pts + 4);
                             armorLight.r2.points(pts);
                             pts8.insert(pts8.cend(), pts, pts + 4);

                             const auto total = cv::minAreaRect(pts8);
//                             CAF_LOG_INFO(fmt::format("A {} {} L {} {} R {} {}",total.size.width,total.size.height, armorLight.r1.size.width,armorLight.r1.size.height,armorLight.r2.size.width,armorLight.r2.size.height));


                             const auto point = solve(cameraMatrix, armorLight);

                             // TODO: projected area
                             res.targets.push_back({ transform(point), 0.0, cars.id });
                         }
                     }
//                     if (!res.targets.empty()) {
//                         auto center = res.targets[0].center.raw();
//                         CAF_LOG_INFO(fmt::format("x: {} y: {} z: {}", center.x, center.y, center.z));
//                     }

                     BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(detect_available_atom_v, mKey);
                 },
                 [&](update_head_atom, Identifier key) { mHeadKey = key; } };
    }
};

HUB_REGISTER_CLASS(ArmorLocator);
