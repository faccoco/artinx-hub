#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SimulatorWorldInfo.hpp"
#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <glm/gtx/matrix_decompose.hpp>
#include <queue>

struct FakeIMUSettings final {
    double delay;
    double imuLinearStd;
    double imuAngularStd;
};

template <class Inspector>
bool inspect(Inspector& f, FakeIMUSettings& x) {
    return f.object(x).fields(f.field("delay", x.delay), f.field("imuLinearStd", x.imuLinearStd),
                              f.field("imuAngularStd", x.imuAngularStd));
}

class FakeIMU final : public HubHelper<caf::event_based_actor, FakeIMUSettings, update_posture_atom> {
    Identifier mKey;
    Duration mDelay;
    std::queue<std::pair<TimePoint, decltype(PostureData::postureOfRobot)>> mQueue;

    std::optional<PostureData> mLastData;

public:
    FakeIMU(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ generateKey(this) }, mDelay{ static_cast<Clock::rep>(
                                                                    mConfig.delay * Clock::period::den / Clock::period::num) } {}
    caf::behavior make_behavior() override {
        return { [&](simulator_step_atom, Identifier key) {
                    ACTOR_PROTOCOL_CHECK(simulator_step_atom, TypedIdentifier<SimulatorWorldInfo>);
                    const auto data = BlackBoard::instance().get<SimulatorWorldInfo>(key).value();
                    mQueue.emplace(data.lastUpdate, data.posture);

                    TimePoint current = mQueue.back().first;

                    std::optional<decltype(PostureData::postureOfRobot)> cur;
                    while(!mQueue.empty() && current - mQueue.front().first > mDelay) {
                        cur = mQueue.front().second;
                        mQueue.pop();
                    }

                    if(!cur.has_value())
                        return;

                    const auto& info = cur.value();

                    PostureData posture;
                    posture.lastUpdate = current;
                    posture.postureOfRobot = info;
                    if(mLastData.has_value()) {
                        const auto& lastData = mLastData.value();
                        const auto dt = Scalar<UnitType::Time>{ static_cast<double>((current - lastData.lastUpdate).count()) /
                                                                Clock::period::den * Clock::period::num };

                        glm::dvec3 scale, translate, translateOld, skew;
                        glm::dvec4 perspective;
                        glm::dquat quat, quatOld;
                        glm::decompose(posture.postureOfRobot.rawInverse(), scale, quat, translate, skew, perspective);
                        glm::decompose(lastData.postureOfRobot.rawInverse(), scale, quatOld, translateOld, skew, perspective);

                        Point<UnitType::Distance, FrameOfReference::Ground> pos{ translate };
                        Point<UnitType::Distance, FrameOfReference::Ground> posOld{ translateOld };

                        Point<UnitType::Angle, FrameOfReference::Ground> angle{ glm::eulerAngles(quat) };
                        Point<UnitType::Angle, FrameOfReference::Ground> angleOld{ glm::eulerAngles(quatOld) };

                        posture.linearVelocityOfRobot = (pos - posOld) / dt;
                        posture.angularVelocityOfRobot = (angle - angleOld) / dt;

                        posture.linearAccelerationOfRobot = (posture.linearVelocityOfRobot - lastData.linearVelocityOfRobot) / dt;
                        posture.angularAccelerationOfRobot =
                            (posture.angularVelocityOfRobot - lastData.angularVelocityOfRobot) / dt;
                    }
                    mLastData = posture;

                    // TODO: add noise

                    sendAll(update_posture_atom_v, BlackBoard::instance().updateSync(mKey, posture));
                },
                 [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); } };
    }
};

HUB_REGISTER_CLASS(FakeIMU);
