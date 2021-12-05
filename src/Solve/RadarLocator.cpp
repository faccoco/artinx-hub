#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "RadarCameraPoints.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <opencv2/calib3d.hpp>

struct RadarLocatorSetting final {};

template <class Inspector>
bool inspect(Inspector& f, RadarLocatorSetting& x) {
    return f.object(x).fields();
}

class RadarLocator final : public HubHelper<caf::event_based_actor, RadarLocatorSetting, radar_locate_succeed_atom> {
    Identifier mKey;
    std::vector<cv::Point3f> mObjectPoints = {
        cv::Point3f(1.51, 7.5, 1.12),  // from rival's base,clockwise
        cv::Point3f(12.897, 1.867, 0.6), cv::Point3f(19.195, 8.612, 0.615),  cv::Point3f(19.195, 9.272, 0.615),
        cv::Point3f(12.03, 10.500, 0.6), cv::Point3f(10.931, 12.546, 1.228),  // gardstation's height unknow, can't find in
                                                                              // manual
        /*coulde add two additional points but may be too many points
         *cv::Point3f(11.446,11.653,0.000),
         *cv::Point3f(11.446,13.44,0.000),
         */
    };

    std::optional<glm::dmat4> locatePosition(const cv::Mat& cameraMatrix, const std::vector<cv::Point2f>& imagePoints,
                                             const Color selfColor) {
        cv::Mat_<double> distCoeff;
        cv::Mat revc, tvec;
        const bool locateSucceed =
            cv::solvePnP(mObjectPoints, imagePoints, cameraMatrix, distCoeff, revc, tvec, false, cv::SOLVEPNP_ITERATIVE);
        if(locateSucceed) {
            cv::Mat rotateMat;
            cv::Rodrigues(revc, rotateMat);
            glm::mat3 rotate{};
            memcpy(glm::value_ptr(rotate), rotateMat.ptr(), sizeof(double) * 3 * 3);
            glm::dmat4 trans = { rotate };

            if(selfColor == Color::Blue) {
                trans[3][0] = tvec.at<double>(0, 0);
                trans[3][1] = tvec.at<double>(1, 0);
                trans[3][2] = tvec.at<double>(2, 0);
            } else {
                trans[3][0] = 28 - tvec.at<double>(0, 0);
                trans[3][1] = 15 - tvec.at<double>(1, 0);
                trans[3][2] = tvec.at<double>(2, 0);
            }
            return trans;
        }
        return std::nullopt;
    }

public:
    RadarLocator(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(RadarLocator).hash_code() } {}
    caf::behavior make_behavior() override {
        return {
            [this](start_atom) {},
            [&](radar_locate_request_atom, Identifier key) {
                const auto data = BlackBoard::instance().get<RadarCameraPointsArray>(key).value();
                const auto& info = data.cameraInfo;
                const cv::Mat cameraMatrix =
                    (cv::Mat_<double>(3, 3) << info.width / 2 / tan(glm::radians(info.fov) / 2), 0, info.width / 2, 0,
                     info.height / 2 / tan(glm::radians(info.fov) / 2), info.height / 2, 0, 0, 1);
                if(const auto radarTransform = locatePosition(cameraMatrix, data.imagePoints, data.selfColor)) {
                    const Transform<FrameOfReference::Camera, FrameOfReference::Ground, true> transform{ radarTransform.value() };
                    BlackBoard::instance().updateSync(mKey, transform);
                    sendAll(radar_locate_succeed_atom_v, mKey);
                }
            }
        };
    }
};

HUB_REGISTER_CLASS(RadarLocator);
