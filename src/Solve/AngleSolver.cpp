#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "ExceptionProbe.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"
#include "Utility.hpp"
#include <cmath>
#include <complex>
#include <limits>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/gtx/string_cast.hpp>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"

static constexpr double minShootTheta = glm::radians(30.0);
static constexpr double maxShootTheta = glm::radians(150.0);
static constexpr double minDelta = 0.001;  // s

struct AngleSolverSettings final {
    double precision;
    double delay;
};

template <class Inspector>
bool inspect(Inspector& f, AngleSolverSettings& x) {
    return f.object(x).fields(f.field("precision", x.precision), f.field("delay", x.delay));
}

class AngleSolver final : public HubHelper<caf::event_based_actor, AngleSolverSettings, set_target_info_atom> {
    Identifier mKey, mIMUKey, mHeadKey;

    const double g, bulletMass, bulletRadius, dragCoefficient, airDensity, delayTime;

    static constexpr glm::dvec3 tf(const glm::dvec3& ori) {
        return { ori.x, -ori.z, ori.y };
    }

public:
    AngleSolver(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ generateKey(this) }, g(GlobalSettings::get().gForce),
          bulletMass(GlobalSettings::get().bulletMass()), bulletRadius(GlobalSettings::get().bulletRadius()),
          dragCoefficient(GlobalSettings::get().dragCoefficient), airDensity(GlobalSettings::get().airDensity),
          delayTime(mConfig.delay) {}

    static std::complex<double> sqrtN(const std::complex<double>& x, double n) {
        if(auto r = std::hypot(x.real(), x.imag()); r > 0.0) {
            auto a = std::atan2(x.imag(), x.real());
            n = 1.0 / n;
            r = std::pow(r, n);
            a *= n;
            return std::polar(r, a);
        }
        return {};
    }

    static double ferrari(std::complex<double> a, std::complex<double> b, std::complex<double> c, std::complex<double> d,
                          std::complex<double> e) {
        std::complex<double> x[4];
        a = 1.0 / a;
        b *= a;
        c *= a;
        d *= a;
        e *= a;
        const auto p = (c * c + 12.0 * e - 3.0 * b * d) / 9.0;
        const auto q = (27.0 * d * d + 2.0 * c * c * c + 27.0 * b * b * e - 72.0 * c * e - 9.0 * b * c * d) / 54.0;
        const auto D = sqrtN(q * q - p * p * p, 2.0);
        std::complex<double> u = q + D;
        std::complex<double> v = q - D;
        if(v.real() * v.real() + v.imag() * v.imag() > u.real() * u.real() + u.imag() * u.imag()) {
            u = sqrtN(v, 3.0);
        } else {
            u = sqrtN(u, 3.0);
        }
        std::complex<double> y;
        if(u.real() * u.real() + u.imag() * u.imag() > 0.0) {
            v = p / u;
            const std::complex<double> o1(-0.5, +0.86602540378443864676372317075294);
            const std::complex<double> o2(-0.5, -0.86602540378443864676372317075294);
            std::complex<double>& yMax = x[0];
            double m2Max = 0.0;
            // int iMax = -1;
            for(int i = 0; i < 3; ++i) {
                y = u + v + c / 3.0;
                u *= o1;
                v *= o2;
                a = b * b + 4.0 * (y - c);
                if(const auto m2 = a.real() * a.real() + a.imag() * a.imag(); 0 == i || m2Max < m2) {
                    m2Max = m2;
                    yMax = y;
                    // iMax = i;
                }
            }
            y = yMax;
        } else {
            y = c / 3.0;
        }
        if(const auto m = sqrtN(b * b + 4.0 * (y - c), 2.0); m.real() * m.real() + m.imag() * m.imag() >= DBL_MIN) {
            const std::complex<double> n = (b * y - 2.0 * d) / m;

            a = sqrtN((b + m) * (b + m) - 8.0 * (y + n), 2.0);
            x[0] = (-(b + m) + a) / 4.0;
            x[1] = (-(b + m) - a) / 4.0;
            a = sqrtN((b - m) * (b - m) - 8.0 * (y - n), 2.0);
            x[2] = (-(b - m) + a) / 4.0;
            x[3] = (-(b - m) - a) / 4.0;
        } else {
            a = sqrtN(b * b - 8.0 * y, 2.0);
            x[0] = x[1] = (-b + a) / 4.0;
            x[2] = x[3] = (-b - a) / 4.0;
        }
        double ans = 1000;
        for(auto& i : x) {
            if(i.real() > 0 && std::fabs(i.imag()) < 1e7 && i.real() < ans)
                ans = i.real();
        }
        return ans;
    }

    // tuple[time,yawAngle,pitchAngle]
    std::tuple<double, double, double> solveWithoutAirDrag(glm::dvec3 targetPos, glm::dvec3 targetVel) {
        constexpr auto square = [](const double x) { return x * x; };
        const double bulletSpeed = GlobalSettings::get().bulletSpeed;
        double airDuration = ferrari(
            1, 0,
            -(4 * g * targetPos.z + 4 * square(bulletSpeed) - 4 * square(targetVel.x) - 4 * square(targetVel.y)) / square(g),
            (8 * targetPos.x * targetVel.x + 8 * targetPos.y * targetVel.y) / square(g),
            (4 * square(targetPos.x) + 4 * square(targetPos.y) + 4 * square(targetPos.z)) / square(g));
        double verticalSpeed = targetPos.z / airDuration - 0.5 * g * airDuration;
        double horizontalSpeedX = (targetPos.x + targetVel.x * airDuration) / airDuration;
        double horizontalSpeedY = (targetPos.y + targetVel.y * airDuration) / airDuration;

        double pitchAngle = std::asin(verticalSpeed / bulletSpeed);
        double yawAngle = std::atan2(horizontalSpeedY, horizontalSpeedX) - glm::half_pi<double>();

        return std::make_tuple(airDuration, yawAngle, pitchAngle);
    }

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
            [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](predict_success_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(predict_success_atom, TypedIdentifier<PredictedTarget>);
                ACTOR_EXCEPTION_PROBE();

                auto data = BlackBoard::instance().get<PredictedTarget>(key);
                if(!(data.has_value()))
                    return;

                Vector<UnitType::Distance, FrameOfRef::Robot> posRefRobot = data->position;
                Vector<UnitType::LinearVelocity, FrameOfRef::Robot> linearVel = data->velocity;
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
                sendAll(set_target_info_atom_v, mGroupMask, data.value().lastUpdate.time_since_epoch().count(), yawAngle,
                        pitchAngle, true);
            },
            [this](outpost_predict_success_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(outpost_predict_success_atom, TypedIdentifier<PredictedOutpost>);
                ACTOR_EXCEPTION_PROBE();

                auto data = BlackBoard::instance().get<PredictedOutpost>(key);
                if(!(data.has_value()))
                    return;

                HubLogger::watch("z", data->centerOfOutpost.mVal.z);

                glm::dvec3 tfCenterPos = tf(data->centerOfOutpost.mVal);
                double predictTime = 0.0667 * glm::sqrt(tfCenterPos.x * tfCenterPos.x + tfCenterPos.y * tfCenterPos.y) +
                    0.0155 * tfCenterPos.z + 0.01;

                for(int i = 0;; i++) {
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
                    glm::dvec3 finalPos = tfCenterPos + glm::dvec3{ glm::cos(theta), -glm::sin(theta), 0 } * radiusOfOutpost;

                    auto [requiredTime, yawAngle, pitchAngle] = solveWithoutAirDrag(finalPos, glm::dvec3{ 0, 0, 0 });

                    if(fabs(predictTime - requiredTime) <= minDelta) {
                        sendAll(set_target_info_atom_v, mGroupMask, data.value().lastUpdate.time_since_epoch().count(), yawAngle,
                                pitchAngle, true);
                        break;
                    } else {
                        logInfo(fmt::format("AngleSolver delta : {}", predictTime - requiredTime));
                        predictTime = requiredTime;
                    }
                }
            },
        };
    }
};

HUB_REGISTER_CLASS(AngleSolver);
