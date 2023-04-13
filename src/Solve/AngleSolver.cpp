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

struct AngleSolverSettings final {
    double precision;
    double delay;
};

template <class Inspector>
bool inspect(Inspector& f, AngleSolverSettings& x) {
    return f.object(x).fields(f.field("precision", x.precision), f.field("delay", x.delay));
}

class AngleSolver final : public HubHelper<caf::event_based_actor, AngleSolverSettings, set_target_info_atom> {
    const double delayTime;

    static constexpr glm::dvec3 tf(const glm::dvec3& ori) {
        return { ori.x, -ori.z, ori.y };
    }

public:
    AngleSolver(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, delayTime(mConfig.delay) {}

    /*
        // tuple[time,yawAngle,pitchAngle]
        std::tuple<double, double, double> solveWithAirDrag(glm::dvec3 targetPos, glm::dvec3 targetVel) {
            double x = targetPos.x, y = targetPos.y, z = targetPos.z;
            double vx = targetVel.x, vy = targetVel.y, vz = targetVel.z;
            double k = 2 * bulletMass / (dragCoefficient * airDensity * bulletRadius * bulletRadius * glm::pi<double>());
            double tmpx = glm::sqrt(k / (k - 2 * x)), tmpy = glm::sqrt(k / (k - 2 * y));
            double a = g * g / 4;
            double b = g * vz;
            double c = vx * vx * tmpx * tmpx * tmpx + vy * vy * tmpy * tmpy * tmpy + g * z + vz * vz - bulletSpeed *
       bulletSpeed; double d = 2 * k * (vx * (tmpx - 1) + vy * (tmpy - 1)) + 2 * vz * z; double e = 2 * k * (k - x - k / tmpx
       + k - y - k / tmpy) + z * z;

            double t = ferrari(a, b, c, d, e);
            double v0x = k * (1 - glm::sqrt(1 - 2 * x / k - 2 * vx * t / k)) / t;
            double v0y = k * (1 - glm::sqrt(1 - 2 * y / k - 2 * vy * t / k)) / t;
            double v0z = g * t / 2 + vz + z / t;

            double pitchAngle = std::asin(v0z / bulletSpeed);
            double yawAngle = std::atan2(v0y, v0x) - glm::half_pi<double>();

            return std::make_tuple(t, yawAngle, pitchAngle);
        }
     */
    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](predict_success_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(predict_success_atom, TypedIdentifier<PredictedTarget>);
                ACTOR_EXCEPTION_PROBE();

                auto data = BlackBoard::instance().get<PredictedTarget>(key);
                if(!(data.has_value()))
                    return;

                Vector<UnitType::Distance, FrameOfRef::Robot> posRefRobot = data->position;
                Vector<UnitType::LinearVelocity, FrameOfRef::Robot> linearVel = data->velocity;
                HubLogger::watch("x", posRefRobot.mVal.x);
                HubLogger::watch("y", posRefRobot.mVal.y);
                HubLogger::watch("z", posRefRobot.mVal.z);

                //(forward:+y,right:+x)
                glm::dvec3 tfPos = tf(posRefRobot.mVal);
                glm::dvec3 tfLinearVel = tf(linearVel.mVal);

                // logInfo(fmt::format("Source Velocity {} {} {}", linearVelocity.raw().x, linearVelocity.raw().y,
                // linearVelocity.raw().z));
                tfPos = { tfPos.x + delayTime * tfLinearVel.x, tfPos.y + delayTime * tfLinearVel.y,
                          tfPos.z + delayTime * tfLinearVel.z };

                auto [time, yawAngle, pitchAngle] = solveWithoutAirDrag(tfPos, tfLinearVel);
                // logInfo(fmt::format("x:{}, y:{}, z:{}", tfPos.x, tfPos.y, tfPos.z));
                sendAllHighPriority(set_target_info_atom_v, mGroupMask, data.value().lastUpdate.time_since_epoch().count(),
                                    yawAngle, pitchAngle, true, normalSolver);
            },
        };
    }
};

HUB_REGISTER_CLASS(AngleSolver);
