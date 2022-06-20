#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "SimulatorWorldInfo.hpp"
#include "Utility.hpp"
#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/random.hpp>
#include <queue>

struct FakeHeadSettings final {
    double delay;
    double headPosStd;
    double headSpeedStd;
    double headMaxSpeed;
    double headHeightOffset;

    double kp, ki, kd;
};

template <class Inspector>
bool inspect(Inspector& f, FakeHeadSettings& x) {
    return f.object(x).fields(f.field("delay", x.delay), f.field("headPosStd", x.headPosStd),
                              f.field("headSpeedStd", x.headSpeedStd), f.field("headMaxSpeed", x.headMaxSpeed),
                              f.field("headHeightOffset", x.headHeightOffset), f.field("kp", x.kp), f.field("ki", x.ki),
                              f.field("kd", x.kd));
}

class FakeHead final : public HubHelper<caf::event_based_actor, FakeHeadSettings, update_head_atom> {
    TimePoint mCurrent = {};
    Duration mDelay;
    std::queue<std::tuple<TimePoint, double, double>> mQueue;

    double mTime = 0.0, mTargetYaw = 0.0, mTargetPitch = 0.0;
    PIDSimulator mYaw, mPitch;
    Identifier mKey;

public:
    FakeHead(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mDelay{ static_cast<Clock::rep>(mConfig.delay * Clock::period::den / Clock::period::num) },
          mYaw{ { mConfig.kp, mConfig.ki, mConfig.kd } }, mPitch{ { mConfig.kp, mConfig.ki, mConfig.kd } }, mKey{
              typeid(FakeHead).hash_code()
          } {
        Timer::instance().addTimer(this->address(), 5ms);
    }
    caf::behavior make_behavior() override {
        return { [&](timer_atom) {
                    ACTOR_PROTOCOL_CHECK(timer_atom);
                    mQueue.push({ mCurrent, mTargetYaw, mTargetPitch });
                },
                 [&](simulator_step_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(simulator_step_atom, TypedIdentifier<SimulatorWorldInfo>);
                     const auto data = BlackBoard::instance().get<SimulatorWorldInfo>(key).value();

                     const auto current = data.lastUpdate;
                     const auto diff =
                         static_cast<double>((current - mCurrent).count()) / Clock::period::den * Clock::period::num;
                     mCurrent = current;

                     std::optional<std::tuple<TimePoint, double, double>> cur;
                     while(!mQueue.empty() && current - std::get<0>(mQueue.front()) > mDelay) {
                         cur = mQueue.front();
                         mQueue.pop();
                     }

                     if(!cur.has_value())
                         return;

                     const auto [_, targetYaw, targetPitch] = cur.value();
                     auto [yaw, yawSpeed] = mYaw.step(diff, targetYaw, mConfig.headMaxSpeed, glm::two_pi<double>());
                     auto [pitch, pitchSpeed] = mPitch.step(diff, targetPitch, mConfig.headMaxSpeed);

                     yaw += glm::gaussRand(0.0, mConfig.headPosStd);
                     pitch += glm::gaussRand(0.0, mConfig.headPosStd);

                     yawSpeed += glm::gaussRand(0.0, mConfig.headSpeedStd);
                     pitchSpeed += glm::gaussRand(0.0, mConfig.headSpeedStd);

                     const HeadInfo info{ mCurrent,
                                          decltype(HeadInfo::transform){ glm::lookAtRH(
                                              glm::dvec3{ 0.0, mConfig.headHeightOffset, 0.0 },
                                              glm::dvec3{ std::cos(yaw - glm::half_pi<double>()) * std::cos(pitch),
                                                          mConfig.headHeightOffset + std::sin(pitch),
                                                          std::sin(yaw - glm::half_pi<double>()) * std::cos(pitch) },
                                              glm::dvec3{ 0.0, 1.0, 0.0 }) },
                                          yawSpeed, pitchSpeed };

                     sendAll(update_head_atom_v, 1U, BlackBoard::instance().updateSync(mKey, info));
                 },
                 [&](set_target_info_atom, GroupMask, Clock::rep, const double yaw, const double pitch, bool isFire) {
                     ACTOR_PROTOCOL_CHECK(set_target_info_atom, GroupMask, Clock::rep, double, double, bool);
                     mTargetYaw = yaw;
                     mTargetPitch = pitch;
                 },
                 [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); } };
    }
};

HUB_REGISTER_CLASS(FakeHead);
