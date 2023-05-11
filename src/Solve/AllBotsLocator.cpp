#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "RadarInfo.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/glm.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>

#include "SuppressWarningEnd.hpp"

struct AllBotsLocatorSettings final {
    float ratioThreshold;
};

template <class Inspector>
bool inspect(Inspector& f, AllBotsLocatorSettings& x) {
    return f.object(x).fields(f.field("ratioThreshold", x.ratioThreshold));
}

class AllBotsLocator final
    : public HubHelper<caf::event_based_actor, AllBotsLocatorSettings, bots_locate_request_atom, image_frame_atom> {
private:
    Identifier mKey /*, mHeadKey{}*/;
    Transform<FrameOfRef::Camera, FrameOfRef::Ground> mTrans;

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
        HubLogger::watch("armorRatio", ratio);

        return ratio > mConfig.ratioThreshold;
    }

    Point<UnitType::Distance, FrameOfRef::Camera> solve([[maybe_unused]] cv::Mat& debugView, const cv::Mat& cameraMatrix,
                                                        bool isLargeArmor) {
        const cv::Mat_<double> distCoeff;
        cv::Mat rvec, tvec;

        [[maybe_unused]] const auto res = cv::solvePnP(isLargeArmor ? mObjectPointsLarge : mObjectPointsSmall, mImagePoint,
                                                       cameraMatrix, distCoeff, rvec, tvec, false, cv::SOLVEPNP_IPPE);
        glm::dvec3 p0 = { tvec.at<double>(0, 0), -tvec.at<double>(1, 0), -tvec.at<double>(2, 0) };

        if(p0.z > 0.0)
            p0 = -p0;

#ifdef ARTINXHUB_DEBUG
        cv::drawFrameAxes(debugView, cameraMatrix, distCoeff, rvec, tvec,
                          static_cast<float>(isLargeArmor ? widthOfLargeArmor : widthOfSmallArmor) * 0.5f);
#endif

        return Point<UnitType::Distance, FrameOfRef::Camera>{ p0 };
    }

public:
    AllBotsLocator(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [this](radar_locate_succeed_atom, Identifier key) {
                     mTrans = BlackBoard::instance().get<RadarTransform>(key).value().trans;
                 } };
    }
};

HUB_REGISTER_CLASS(AllBotsLocator);
