#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "ExceptionProbe.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/gtx/string_cast.hpp>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"

static constexpr double minShootTheta = glm::radians(30.0);
static constexpr double maxShootTheta = glm::radians(150.0);
static constexpr double minDelta = 0.001;  // s
static constexpr int maxCycleTimes = 5;

struct OutpostSolverSettings final {
    double precision;
    double delay;
};

template <class Inspector>
bool inspect(Inspector& f, OutpostSolverSettings& x) {
    return f.object(x).fields(f.field("precision", x.precision), f.field("delay", x.delay));
}

class OutpostSolver final : public HubHelper<caf::event_based_actor, OutpostSolverSettings, set_target_info_atom> {
    const double delayTime;

    static constexpr glm::dvec3 tf(const glm::dvec3& ori) {
        return { ori.x, -ori.z, ori.y };
    }

public:
    OutpostSolver(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, delayTime(mConfig.delay) {}
    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](outpost_predict_success_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(outpost_predict_success_atom, TypedIdentifier<PredictedOutpost>);
                ACTOR_EXCEPTION_PROBE();

                auto data = BlackBoard::instance().get<PredictedOutpost>(key);
                if(!(data.has_value()))
                    return;

                HubLogger::watch("center z", data->centerOfOutpost.mVal.z);

                glm::dvec3 tfCenterPos = tf(data->centerOfOutpost.mVal);
                double predictTime = 0.0667 * glm::sqrt(tfCenterPos.x * tfCenterPos.x + tfCenterPos.y * tfCenterPos.y) +
                    0.0155 * tfCenterPos.z + 0.01;

                for(int i = 0; i < maxCycleTimes; i++) {
                    double theta = data->theta.mVal + (predictTime + delayTime) * data->angularVelocity.mVal;
                    if(theta > maxShootTheta) {
                        theta -= glm::radians<double>(120);
                        if(theta < minShootTheta)
                            return;
                    } else if(theta < minShootTheta) {
                        theta += glm::radians<double>(120);
                        if(theta > maxShootTheta)
                            return;
                    }
                    glm::dvec3 finalPos = tfCenterPos + glm::dvec3{ glm::cos(theta), -glm::sin(theta), 0 } * data->radius.mVal;

                    auto [requiredTime, yawAngle, pitchAngle] = solveWithoutAirDrag(finalPos, glm::dvec3{ 0, 0, 0 });

                    if(fabs(predictTime - requiredTime) <= minDelta) {
                        sendAll(set_target_info_atom_v, mGroupMask, data.value().lastUpdate.time_since_epoch().count(), yawAngle,
                                pitchAngle, true, normalSolver);
                        break;
                    } else {
                        // logInfo(fmt::format("OutpostSolver delta : {}", predictTime - requiredTime));
                        predictTime = requiredTime;
                    }
                }
            },
        };
    }
};

HUB_REGISTER_CLASS(OutpostSolver);
