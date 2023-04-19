#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "RadarCameraPoints.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <opencv2/calib3d.hpp>

#include "SuppressWarningEnd.hpp"

struct RadarLocatorSetting final {};

template <class Inspector>
bool inspect(Inspector& f, RadarLocatorSetting& x) {
    return f.object(x).fields();
}

class RadarLocator final : public HubHelper<caf::event_based_actor, RadarLocatorSetting, radar_locate_succeed_atom> {
    Identifier mKey;
    const std::vector<cv::Point3f> mObjectPoints = {
        cv::Point3f(1.51f, 7.5f, 1.12f),  // from rival's base,clockwise
        cv::Point3f(12.897f, 1.867f, 0.6f), cv::Point3f(19.195f, 8.612f, 0.615f),  cv::Point3f(19.195f, 9.272f, 0.615f),
        cv::Point3f(12.03f, 10.500f, 0.6f), cv::Point3f(10.931f, 12.546f, 1.228f),  // guardStation's height unknown, can't find
                                                                                    // in manual
        /*could add two additional points but may be too many points
         *cv::Point3f(11.446,11.653,0.000),
         *cv::Point3f(11.446,13.44,0.000),
         */
    };

    std::optional<RadarTransform> locatePosition(const cv::Mat& cameraMatrix, const std::vector<cv::Point2f>& imagePoints) {
        const cv::Mat_<double> distCoeff;
        std::vector<double> rvec, tvec;
        if(cv::solvePnP(mObjectPoints, imagePoints, cameraMatrix, distCoeff, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
            cv::Mat rotateMat;
            cv::Rodrigues(rvec, rotateMat);
            glm::dmat3 rotate{};
            memcpy(glm::value_ptr(rotate), rotateMat.ptr(), sizeof(double) * 3 * 3);
            glm::dmat4 trans = { rotate };
            auto&& selfColor = GlobalSettings::get().selfColor;
            if(selfColor == Blue) {
                trans[3][0] = tvec[0];
                trans[3][1] = tvec[1];
            } else {
                trans[3][0] = 28 - tvec[0];
                trans[3][1] = 15 - tvec[1];
            }
            trans[3][2] = tvec[2];
            logInfo(fmt::format("current color {}", selfColor == Blue ? "Blue" : "Red"));
            logInfo(fmt::format("trans: {} {} {}", trans[3][0], trans[3][1], trans[3][2]));
            return { { glm::inverse(trans), rotate } };
        }
        return std::nullopt;
    }

public:
    RadarLocator(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [this](radar_locate_request_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(radar_locate_request_atom, TypedIdentifier<RadarCameraPoints>);
                     ACTOR_EXCEPTION_PROBE();

                     const auto data = BlackBoard::instance().get<RadarCameraPoints>(key).value();
                     if(const auto radarTransform = locatePosition(data.info.cameraMatrix, data.points)) {
                         sendAll(radar_locate_succeed_atom_v,
                                 BlackBoard::instance().updateSync(mKey, std::move(radarTransform.value())));
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(RadarLocator);
