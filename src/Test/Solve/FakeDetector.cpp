#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedTarget.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "SimulatorWorldInfo.hpp"
#include <cstdint>
#include <queue>

#include "SuppressWarningBegin.hpp"

#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <glm/gtc/random.hpp>

#include "SuppressWarningEnd.hpp"

struct FakeDetectorSettings final {
    int fps;
    double delay;
    double detectLinearStd;
};

template <class Inspector>
bool inspect(Inspector& f, FakeDetectorSettings& x) {
    return f.object(x).fields(f.field("fps", x.fps).fallback(100), f.field("delay", x.delay).fallback(0.0),
                              f.field("detectLinearStd", x.detectLinearStd).fallback(0.0));
}

class FakeDetector final : public HubHelper<caf::event_based_actor, FakeDetectorSettings, detect_available_atom> {
    Identifier mKey, mHeadKey{};
    std::optional<TimePoint> mFirstReceive;
    float mReceivedTimes;
    Duration mDelay;
    std::queue<SimulatorWorldInfo> mQueue;

public:
    FakeDetector(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, name }, mKey{ generateKey(this) }, mReceivedTimes(0), mDelay(doubleCastDuration(mConfig.delay)) {}
    caf::behavior make_behavior() override {
        return { [&](simulator_step_atom, Identifier key) {
                    ACTOR_PROTOCOL_CHECK(simulator_step_atom, TypedIdentifier<SimulatorWorldInfo>);
                    auto newInfo = BlackBoard::instance().get<SimulatorWorldInfo>(key).value();

                    if(!mFirstReceive.has_value())
                        mFirstReceive = newInfo.lastUpdate;

                    if(newInfo.lastUpdate - mFirstReceive.value() > doubleCastDuration(mReceivedTimes / mConfig.fps)) {
                        mQueue.push(newInfo);
                        mReceivedTimes += 1;
                    } else {
                        return;
                    }

                    const auto headData = BlackBoard::instance().get<HeadInfo>(mHeadKey);
                    if(!headData.has_value())
                        return;

                    std::optional<SimulatorWorldInfo> cur;
                    while(!mQueue.empty() && mQueue.back().lastUpdate - mQueue.front().lastUpdate > mDelay) {
                        cur = mQueue.front();
                        mQueue.pop();
                    }

                    if(!cur.has_value())
                        return;

                    const auto& info = cur.value();
                    const auto& head = headData.value();

                    DetectedTargetArray data;

                    data.lastUpdate = cur->lastUpdate;

                    const auto tfGround2Gun = combine(info.tfGround2Robot, head.tfRobot2Gun);

                    for(const auto& [target, tfArmor2Ground] : info.targets) {
                        const auto pos = tfGround2Gun(target);
                        const auto rmat = combine(tfGround2Gun, tfArmor2Ground);
                        auto noise = glm::zero<glm::dvec3>();
                        if(mConfig.detectLinearStd > 1e-3) {
                            noise = glm::gaussRand(glm::zero<glm::dvec3>(), glm::dvec3{ mConfig.detectLinearStd });
                            noise = glm::clamp(noise, glm::dvec3{ -3.0 * mConfig.detectLinearStd },
                                               glm::dvec3{ 3.0 * mConfig.detectLinearStd });
                        }

                        data.targets.push_back(
                            DetectedTarget{ { static_cast<float>(pos.mVal.x) * 1e9f, static_cast<float>(pos.mVal.z) * 1e9f },
                                            length((pos + Vector<UnitType::Distance, FrameOfRef::Gun>(noise)).mVal),
                                            pos + Vector<UnitType::Distance, FrameOfRef::Gun>(noise),
                                            RobotType::Infantry1,
                                            ArmorType::Small,
                                            ArmorMotion::Unsure,
                                            rmat });
                    }

                    sendAll(detect_available_atom_v, mGroupMask, BlackBoard::instance().updateSync(mKey, data));
                },
                 [&](update_head_atom, GroupMask, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(update_head_atom, GroupMask, TypedIdentifier<HeadInfo>);
                     mHeadKey = key;
                 },
                 [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); } };
    }
};

HUB_REGISTER_CLASS(FakeDetector);
