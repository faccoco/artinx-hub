#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"
#include "Utility.hpp"
#include <caf/event_based_actor.hpp>
#include <cmath>
#include <complex>
#include <magic_enum.hpp>

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

private:
    double mPositions[1005] = {};
    double mTimes[1005] = {};
    double mDiff[1005] = {};
    double mPeriod = 0;
    int mExceptionPoint[1005] = {};
    int mCnt = 0;
    double mPreviousYawAngle;
    double mPreviousPitchAngle;

public:
    AngleSolver(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(AngleSolver).hash_code() } {}

    static std::complex<double> sqrtn(const std::complex<double>& x, double n) {
        if(double r = std::hypot(x.real(), x.imag()); r > 0.0) {

            double a = std::atan2(x.imag(), x.real());
            n = 1.0 / n;
            r = std::pow(r, n);
            a *= n;
            return std::complex<double>(r * std::cos(a), r * std::sin(a));
        }
        return std::complex<double>();
    };

    static double Ferrari(std::complex<double> a, std::complex<double> b, std::complex<double> c, std::complex<double> d,
                          std::complex<double> e) {
        std::complex<double> x[4];
        a = 1.0 / a;
        b *= a;
        c *= a;
        d *= a;
        e *= a;
        const auto p = (c * c + 12.0 * e - 3.0 * b * d) / 9.0;
        const auto q = (27.0 * d * d + 2.0 * c * c * c + 27.0 * b * b * e - 72.0 * c * e - 9.0 * b * c * d) / 54.0;
        const auto D = sqrtn(q * q - p * p * p, 2.0);
        std::complex<double> u = q + D;
        std::complex<double> v = q - D;
        if(v.real() * v.real() + v.imag() * v.imag() > u.real() * u.real() + u.imag() * u.imag()) {
            u = sqrtn(v, 3.0);
        } else {
            u = sqrtn(u, 3.0);
        }
        std::complex<double> y;
        if(u.real() * u.real() + u.imag() * u.imag() > 0.0) {
            v = p / u;
            const std::complex<double> o1(-0.5, +0.86602540378443864676372317075294);
            const std::complex<double> o2(-0.5, -0.86602540378443864676372317075294);
            std::complex<double>& yMax = x[0];
            double m2 = 0.0;
            double m2Max = 0.0;
            int iMax = -1;
            for(int i = 0; i < 3; ++i) {
                y = u + v + c / 3.0;
                u *= o1;
                v *= o2;
                a = b * b + 4.0 * (y - c);
                m2 = a.real() * a.real() + a.imag() * a.imag();
                if(0 == i || m2Max < m2) {
                    m2Max = m2;
                    yMax = y;
                    iMax = i;
                }
            }
            y = yMax;
        } else {
            y = c / 3.0;
        }
        if(const auto m = sqrtn(b * b + 4.0 * (y - c), 2.0); m.real() * m.real() + m.imag() * m.imag() >= DBL_MIN) {
            const std::complex<double> n = (b * y - 2.0 * d) / m;

            a = sqrtn((b + m) * (b + m) - 8.0 * (y + n), 2.0);
            x[0] = (-(b + m) + a) / 4.0;
            x[1] = (-(b + m) - a) / 4.0;
            a = sqrtn((b - m) * (b - m) - 8.0 * (y - n), 2.0);
            x[2] = (-(b - m) + a) / 4.0;
            x[3] = (-(b - m) - a) / 4.0;
        } else {
            a = sqrtn(b * b - 8.0 * y, 2.0);
            x[0] = x[1] = (-b + a) / 4.0;
            x[2] = x[3] = (-b - a) / 4.0;
        }
        double ans = 0;
        for(auto& i : x) {
            if(i.real() > ans && std::abs(i.imag()) < 1e7)
                ans = i.real();
        }
        return ans;
    }

    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [this](set_target_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(set_target_atom, TypedIdentifier<SelectedTarget>);
                     const auto data = BlackBoard::instance().get<SelectedTarget>(key);
                     const auto dataHeadInfo = BlackBoard::instance().get<HeadInfo>(mHeadKey);
                     const auto dataPosture = BlackBoard::instance().get<PostureData>(mIMUKey);
                     if(!(data.has_value() && data.value().selected.has_value() && dataHeadInfo.has_value() &&
                          dataPosture.has_value()))
                         return;
                     HubLogger::watch("armor type", magic_enum::enum_name(data.value().selected.value().type));

                     const auto& globalSettings = GlobalSettings::get();
                     const double g = -globalSettings.gForce, bulletSpeed = globalSettings.bulletSpeed;

                     constexpr auto square = [=](const double x) { return x * x; };

                     Vector<UnitType::Distance, FrameOfReference::Gun> positionOfReferenceGun(
                         data.value().selected.value().center.raw());
                     const auto timeDuration = static_cast<double>(data.value().lastUpdate.time_since_epoch().count()) / 1e9;

                     const auto delayTime = -mConfig.delay;

                     glm::dvec3 transformedPosition = { 0, 0, 0 };
                     glm::dvec3 forwardVector = { 0, 0, -1 };
                     glm::dvec3 transformedLinearVelocity = { 0, 0, 0 };

                     Vector<UnitType::Distance, FrameOfReference::Gun> forwardPosition(forwardVector);
                     Vector<UnitType::Distance, FrameOfReference::Robot> positionOfReferenceRobot =
                         dataHeadInfo.value().transform(positionOfReferenceGun);
                     Vector<UnitType::Distance, FrameOfReference::Ground> positionOfReferenceGround =
                         dataPosture.value().postureOfRobot(positionOfReferenceRobot);
                     HubLogger::watch("z", positionOfReferenceGround.raw().z);

                     Vector<UnitType::Distance, FrameOfReference::Robot> forwardPositionOfReferenceRobot =
                         dataHeadInfo.value().transform(forwardPosition);
                     Vector<UnitType::Distance, FrameOfReference::Ground> forwardPositionOfReferenceGround =
                         dataPosture.value().postureOfRobot(forwardPositionOfReferenceRobot);
                     Vector<UnitType::LinearVelocity, FrameOfReference::Ground> linearVelocity(
                         dataPosture.value().linearVelocityOfRobot.raw());
                     transformedPosition = { positionOfReferenceGround.raw().x, -positionOfReferenceGround.raw().z,
                                             positionOfReferenceGround.raw().y };
                     forwardVector = { forwardPositionOfReferenceGround.raw().x, -forwardPositionOfReferenceGround.raw().z,
                                       forwardPositionOfReferenceGround.raw().y };
                     transformedLinearVelocity = { linearVelocity.raw().x, -linearVelocity.raw().z, linearVelocity.raw().y };

                     transformedPosition = { transformedPosition.x + delayTime * transformedLinearVelocity.x,
                                             transformedPosition.y + delayTime * transformedLinearVelocity.y,
                                             transformedPosition.z + delayTime * transformedLinearVelocity.z };
                     //(forward:+y,right:+x)
                     mTimes[(++mCnt) % 1000] = timeDuration;
                     mPositions[(mCnt) % 1000] = transformedPosition.x;
                     if(mCnt >= 2) {
                         mDiff[(mCnt - 1) % 1000] = (mPositions[mCnt % 1000] - mPositions[(mCnt - 1) % 1000]) -
                             (mTimes[mCnt % 1000] - mTimes[(mCnt - 1) % 1000]);
                     }
                     if(mCnt >= 3) {
                         if(std::abs(mDiff[(mCnt - 1) % 1000] - mDiff[(mCnt - 2) % 1000]) > 0.2 &&
                            std::abs(mDiff[(mCnt - 1) % 1000] - mDiff[mCnt % 1000]) > 0.2) {
                             mExceptionPoint[(++mExceptionPoint[0]) % 1000] = (mCnt - 1) % 1000;
                             // if(mExceptionPoint[0] >= 2) {
                             //     if(mExceptionPoint[0] == 2)
                             //         mPeriod = mTimes[mExceptionPoint[2]] - mTimes[mExceptionPoint[1]];
                             //     else {
                             //         mPeriod =
                             //             (mPeriod * (mExceptionPoint[0] - 2) + mTimes[mExceptionPoint[mExceptionPoint[0]]]
                             //             -
                             //              mTimes[mExceptionPoint[mExceptionPoint[0] - 1]]) /
                             //             (mExceptionPoint[0] - 1);
                             //     }
                             // }
                         } else
                             mExceptionPoint[0] = 0;
                     }

                     // CAF_LOG_INFO(fmt::format("mCnt:{}mPeriod:{} ", mCnt, mPeriod));
                     // if(!(mCnt%10000))
                     // for(int i = 1; i <= mCnt; ++i) {
                     //    CAF_LOG_INFO(fmt::format("mCnt:{} time: {} transformedposition: {}, mDiff:{}, mPeriod:{} ", i,
                     //    mTimes[i], mPositions[i], mDiff[i], mPeriod));
                     //}
                     // CAF_LOG_INFO(fmt::format("time: {} transformedposition: {} {} {}", timeDuration, transformedPosition.x
                     // ,transformedPosition.y ,transformedPosition.z));
                     double horizonalDistance = std::hypot(transformedPosition.x, transformedPosition.y);
                     double yawAngle = std::atan2(transformedPosition.y, transformedPosition.x) - glm::half_pi<double>();
                     if(yawAngle < 0)
                         yawAngle += glm::two_pi<double>();
                     double netHorizonalSpeed =
                         Ferrari(-square(horizonalDistance) - square(transformedPosition.z),
                                 2 * std::cos(yawAngle) * square(horizonalDistance) * transformedLinearVelocity.x +
                                     2 * std::sin(yawAngle) * square(horizonalDistance) * transformedLinearVelocity.y,
                                 -g * square(horizonalDistance) * transformedPosition.z +
                                     square(horizonalDistance) * square(bulletSpeed) -
                                     square(horizonalDistance) *
                                         (square(transformedLinearVelocity.x) + square(transformedLinearVelocity.y)),
                                 0, (-0.25) * square(g) * square(square(horizonalDistance)));
                     //
                     //                 CAF_LOG_INFO(fmt::format("mCnt:{} ag:{} ", mCnt,
                     //                 std::acos(netHorizonalSpeed/bulletSpeed))); const auto expr1 =
                     //                 std::sqrt(square(bulletSpeed)* square(bulletSpeed) - 2 * g * transformedPosition.z *
                     //                 square(bulletSpeed) - square(horizonalDistance * g) ) / square(g); const auto expr2 =
                     //                 std::sqrt(square(bulletSpeed / g) - transformedPosition.z / g - expr1); auto expr3 = 2
                     //                 * glm::root_two<double>() * horizonalDistance * ((-2 * g * transformedPosition.z + 2 *
                     //                 std::pow(bulletSpeed, 2)) * expr2 - square(g) * cube(expr2)); expr3 /= (double)(4 *
                     //                 bulletSpeed); expr3 /= (square(transformedPosition.x) + square(transformedPosition.y)
                     //                 +
                     //                           square(transformedPosition.z));
                     //                 auto expr4 = 2 * glm::root_two<double>() *
                     //                     (-square(horizonalDistance) * g * expr2 + square(transformedPosition.z) * g *
                     //                     expr2 -
                     //                      2 * transformedPosition.z * square(bulletSpeed) * expr2 +
                     //                      square(g) * transformedPosition.z * cube(expr2));
                     //                 expr4 /= (double)((-4) * bulletSpeed);
                     //                 expr4 /= (square(transformedPosition.x) + square(transformedPosition.y) +
                     // square(transformedPosition.z));
                     //                 double pitchAngle = std::atan2(expr4, expr3);
                     double airDuration = horizonalDistance / netHorizonalSpeed;
                     double netVerticalSpeed = transformedPosition.z / airDuration + g * airDuration / 2;
                     double pitchAngle = std::asin(netVerticalSpeed / bulletSpeed);
                     pitchAngle = (pitchAngle > glm::quarter_pi<double>()) ? (glm::half_pi<double>() - pitchAngle) : pitchAngle;

                     if(mExceptionPoint[0] >= 7) {
                         pitchAngle = mPreviousPitchAngle;
                         yawAngle = mPreviousYawAngle;
                     } else {
                         mPreviousPitchAngle = pitchAngle;
                         mPreviousYawAngle = yawAngle;
                     }

                     // double currentYawAngle = std::atan2(forwardVector.y, forwardVector.x) - glm::half_pi<double>();
                     // double currentPitchAngle = std::atan2(forwardVector.z, std::hypot(forwardVector.x, forwardVector.y));
                     //  CAF_LOG_INFO(fmt::format("CurrentYawAngle: {},CurrentPitchAngle: {}",currentYawAngle,
                     //  currentPitchAngle)); double prec = mConfig.precision; double prec = 0.001;
                     //  CAF_LOG_INFO(fmt::format("Prec:{}", prec));
                     //  bool ifShoot = ((mExceptionPoint[0] >= 3))||(
                     //     (std::abs(currentPitchAngle - pitchAngle) < prec) &&
                     //     ((std::abs(currentYawAngle - yawAngle) < prec) ||
                     //      (std::abs(currentYawAngle - glm::half_pi<double>() - yawAngle) < prec)));
                     sendAll(set_target_info_atom_v, static_cast<Clock::rep>(data->lastUpdate.time_since_epoch().count()),
                             yawAngle, pitchAngle, true);
                 },
                 [this](update_head_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(update_head_atom, TypedIdentifier<HeadInfo>);
                     mHeadKey = key;
                 },
                 [this](update_posture_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(update_posture_atom, TypedIdentifier<PostureData>);
                     mIMUKey = key;
                 } };
    }
};

HUB_REGISTER_CLASS(AngleSolver);
