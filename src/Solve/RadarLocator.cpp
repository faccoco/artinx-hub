#ifdef ARTINX_RADAR
#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "RadarInfo.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <glm/fwd.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>

#include "SuppressWarningEnd.hpp"

class RadarLocator final : public HubHelper<caf::event_based_actor, void, radar_locate_succeed_atom> {
    Identifier mKey;
    const std::vector<cv::Point3f> ObjectPoints = {
        cv::Point3f(7.5f, 1.51f, 1.12f), cv::Point3f(6.39f, 19.195, 0.615f),
        cv::Point3f(5.73f, 19.195f, 0.615f),                                              // from rival's base,clockwise
        /*  cv::Point3f(12.03f, 10.500f, 0.6f),*/ cv::Point3f(2.4532f, 10.931f, 1.228f),  // guardStation's height unknown, can't
        /*cv::Point3f(11.446,11.653,0.000),
         *cv::Point3f(11.446,13.44,0.000),
         */
    };
    const std::vector<cv::Point2f> PerspectPoints = { cv::Point2f(570, 1919.5), cv::Point2f(639, 1919.5), cv::Point2f(335, 1145),
                                                      cv::Point2f(156, 1145) };

    std::optional<glm::dmat4> solveTransform(const RadarCameraPoints& info) {
        const cv::Mat_<double> distCoeff;
        std::vector<double> rvec, tvec;
        if(cv::solvePnP(ObjectPoints, info.points, info.frame.cameraMatrix, distCoeff, rvec, tvec, false,
                        cv::SOLVEPNP_ITERATIVE)) {
            cv::Mat rotateMat;
            cv::Rodrigues(rvec, rotateMat);
            glm::dmat3 rotate{};
            memcpy(glm::value_ptr(rotate), rotateMat.ptr(), sizeof(double) * 3 * 3);
            glm::dmat4 trans = { rotate };
            auto&& selfColor = GlobalSettings::get().getColor();
            if(selfColor == Color::Blue) {
                trans[3][0] = tvec[0];
                trans[3][1] = tvec[1];
            } else {
                trans[3][0] = 28 - tvec[0];
                trans[3][1] = 15 - tvec[1];
            }
            trans[3][2] = tvec[2];
            trans[3][3] = 1;
            return { glm::inverse(trans) };
        }
        return std::nullopt;
    }
    std::optional<cv::Mat> solvePerspectTransform(const RadarCameraPoints& info) {
        return { cv::getPerspectiveTransform(info.points, PerspectPoints) };
        return std::nullopt;
    }

public:
    RadarLocator(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [this](radar_locate_request_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(radar_locate_request_atom, TypedIdentifier<RadarCameraPoints>);

                     if(const auto data = BlackBoard::instance().get<RadarCameraPoints>(key)) {
                         // if(const auto radarTransform = solveTransform(data.value())) {
                         // RadarTransform::instant().set(radarTransform.value());
                         // sendAll(radar_locate_succeed_atom_v);
                         //}

                         if(const auto perspectTransform = solvePerspectTransform(data.value())) {
                             RadarPerspectTransform::instant().store(perspectTransform.value());
                             RadarPerspectTransform::instant().setReady();
                             sendAll(radar_locate_succeed_atom_v);
                         }
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(RadarLocator);
#endif
