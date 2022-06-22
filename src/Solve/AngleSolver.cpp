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

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
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
    Identifier mKey, mIMUKey, mHeadKey;

private:
    glm::dvec3 mPositions[1005] = {};
    double mTimes[1005] = {};
    glm::dvec3 mDiff[1005] = {};
    glm::dvec3 mVec[1005] = {};
    glm::dvec3 sumVec = { 0, 0, 0 };
    glm::dvec3 avgVec = { 0, 0, 0 };
    double mPeriod = 0;
    int mExceptionPoint[1005] = {};
    int mCnt = 0;
    int emptyData = 0;
    double mPreviousYawAngle;
    double mPreviousPitchAngle;

public:
    AngleSolver(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}

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
        double ans = 0;
        for(auto& i : x) {
            if(i.real() > ans && std::fabs(i.imag()) < 1e7)
                ans = i.real();
        }
        return ans;
    }

    caf::behavior make_behavior() override {
        return {
            [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](set_target_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(set_target_atom, TypedIdentifier<SelectedTarget>);
                ACTOR_LATENCY_PROBE();

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

                const auto delayTime = mConfig.delay;

                glm::dvec3 forwardVector = { 0, 0, -1 };
                glm::dvec3 transformedLinearVelocity = { 0, 0, 0 };

                Vector<UnitType::Distance, FrameOfReference::Gun> forwardPosition(forwardVector);
                Vector<UnitType::Distance, FrameOfReference::Robot> positionOfReferenceRobot =
                    dataHeadInfo.value().transform(positionOfReferenceGun);
                Vector<UnitType::Distance, FrameOfReference::Ground> positionOfReferenceGround =
                    dataPosture.value().postureOfRobot(positionOfReferenceRobot);
                HubLogger::watch("z", positionOfReferenceGround.raw().z);
                glm::dvec3 transformedPosition = { positionOfReferenceGround.raw().x, -positionOfReferenceGround.raw().z,
                                                   positionOfReferenceGround.raw().y };

                Vector<UnitType::Distance, FrameOfReference::Robot> forwardPositionOfReferenceRobot =
                    dataHeadInfo.value().transform(forwardPosition);
                Vector<UnitType::Distance, FrameOfReference::Ground> forwardPositionOfReferenceGround =
                    dataPosture.value().postureOfRobot(forwardPositionOfReferenceRobot);
                Vector<UnitType::LinearVelocity, FrameOfReference::Ground> linearVelocity(
                    dataPosture.value().linearVelocityOfRobot.raw());

                
                forwardVector = { forwardPositionOfReferenceGround.raw().x, -forwardPositionOfReferenceGround.raw().z,
                                  forwardPositionOfReferenceGround.raw().y };
                transformedLinearVelocity = { linearVelocity.raw().x, -linearVelocity.raw().z, linearVelocity.raw().y };
                logInfo(
                    fmt::format("Linear v {} {} {}", linearVelocity.raw().x, linearVelocity.raw().y, linearVelocity.raw().z));
                transformedPosition = { transformedPosition.x - delayTime * transformedLinearVelocity.x,
                                        transformedPosition.y - delayTime * transformedLinearVelocity.y,
                                        transformedPosition.z - delayTime * transformedLinearVelocity.z };
                //(forward:+y,right:+x)

                double horizontalDistance = std::hypot(transformedPosition.x, transformedPosition.y);
                double theta = std::atan2(transformedPosition.y, transformedPosition.x);
                double netHorizontalSpeed = ferrari(
                    -square(horizontalDistance) - square(transformedPosition.z),
                    2 * std::cos(theta) * square(horizontalDistance) * transformedLinearVelocity.x +
                        2 * std::sin(theta) * square(horizontalDistance) * transformedLinearVelocity.y,
                    -g * square(horizontalDistance) * transformedPosition.z + square(horizontalDistance) * square(bulletSpeed) -
                        square(horizontalDistance) * (square(transformedLinearVelocity.x) + square(transformedLinearVelocity.y)),
                    0, (-0.25) * square(g) * square(square(horizontalDistance)));
                double airDuration = horizontalDistance / netHorizontalSpeed;
                mTimes[(++mCnt) % 1000] = timeDuration;
                mPositions[(mCnt) % 1000] = transformedPosition;
                if(mCnt >= 2) {
                    if(mTimes[mCnt % 1000] - mTimes[(mCnt - 1) % 1000] > 1e-8) {
                        mVec[(mCnt - 1) % 1000] = { 0, 0, 0 };
                        emptyData += 1;
                    } else {
                        mVec[(mCnt - 1) % 1000] = (mPositions[mCnt % 1000] - mPositions[(mCnt - 1) % 1000]) /
                            (mTimes[mCnt % 1000] - mTimes[(mCnt - 1) % 1000]);
                        sumVec += mVec[(mCnt - 1) % 1000];
                    }

                    if(mCnt >= 3 &&
                       std::fabs((glm::length(mVec[(mCnt - 1) % 1000] - mVec[(mCnt - 2) % 1000]))) > 0.2 &&
                       std::fabs((glm::length(mVec[(mCnt - 1) % 1000] - mVec[mCnt % 1000]))) > 0.2) {
                        mExceptionPoint[(mCnt - 1) % 1000] = 1;
                        mExceptionPoint[0]++;
                    }

                    if(mCnt >= 502) {
                        if(glm::length(mVec[(mCnt - 501) % 1000]) < 1e-8)
                            emptyData -= 1;
                        if(mExceptionPoint[(mCnt - 1) % 1000] == 1) {
                            mExceptionPoint[0]--;
                            mExceptionPoint[(mCnt - 1) % 1000] = 0;
                        }

                        sumVec -= mVec[(mCnt - 501) % 1000];
                        if(emptyData < 500)
                            avgVec = sumVec / static_cast<double>(500 - emptyData);
                        else
                            avgVec = { 0, 0, 0 };
                        //logInfo(fmt::format("Linear v {} {} {}", avgVec.x, avgVec.y, avgVec.z));
                        transformedPosition = { transformedPosition.x + airDuration * avgVec.x ,
                                                transformedPosition.y + airDuration * avgVec.y ,
                                                transformedPosition.z + airDuration * avgVec.z 
                        };
                    }
                }
                netHorizontalSpeed = ferrari(
                    -square(horizontalDistance) - square(transformedPosition.z),
                    2 * std::cos(theta) * square(horizontalDistance) * transformedLinearVelocity.x +
                        2 * std::sin(theta) * square(horizontalDistance) * transformedLinearVelocity.y,
                    -g * square(horizontalDistance) * transformedPosition.z + square(horizontalDistance) * square(bulletSpeed) -
                        square(horizontalDistance) * (square(transformedLinearVelocity.x) + square(transformedLinearVelocity.y)),
                    0, (-0.25) * square(g) * square(square(horizontalDistance)));
                double netVerticalSpeed = transformedPosition.z / airDuration + g * airDuration / 2;
                double pitchAngle = std::asin(netVerticalSpeed / bulletSpeed);
                const auto vy = netHorizontalSpeed * std::sin(theta) - transformedLinearVelocity.y;
                const auto vx = netHorizontalSpeed * std::cos(theta) - transformedLinearVelocity.x;
                double yawAngle = std::atan2(vy, vx);
                yawAngle -= glm::half_pi<double>();

                pitchAngle = (pitchAngle > glm::quarter_pi<double>()) ? (glm::half_pi<double>() - pitchAngle) : pitchAngle;

                if(mExceptionPoint[0] >= 7) {
                    pitchAngle = mPreviousPitchAngle;
                    yawAngle = mPreviousYawAngle;
                } else {
                    mPreviousPitchAngle = pitchAngle;
                    mPreviousYawAngle = yawAngle;
                }
                double currentYawAngle = std::atan2(forwardVector.y, forwardVector.x);
                currentYawAngle -= glm::half_pi<double>();

                double currentPitchAngle = std::atan2(forwardVector.z, std::hypot(forwardVector.x, forwardVector.y));
                // TODO: variant tolerance: size / sin(2*theta) \approx dist
                const auto diffAngle = [](const double a, const double b) {
                    const auto delta = std::fabs(a - b);
                    assert(delta < glm::two_pi<double>());
                    return std::min(delta, glm::two_pi<double>() - delta);
                };

                bool ifShoot = ((diffAngle(yawAngle, currentYawAngle) < mConfig.precision) &&
                    (diffAngle(pitchAngle, currentPitchAngle) < mConfig.precision)) ||
                    (mExceptionPoint[0] >= 7);

                sendAll(set_target_info_atom_v, mGroupMask, data.value().lastUpdate.time_since_epoch().count(), yawAngle,
                        pitchAngle, ifShoot);
            },
            [this](update_head_atom, GroupMask, Identifier key) {
                ACTOR_PROTOCOL_CHECK(update_head_atom, GroupMask, TypedIdentifier<HeadInfo>);
                mHeadKey = key;
            },
            [this](update_posture_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(update_posture_atom, TypedIdentifier<PostureData>);
                mIMUKey = key;
            }
        };
    }
};

HUB_REGISTER_CLASS(AngleSolver);
