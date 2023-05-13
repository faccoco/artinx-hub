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

struct PeriodSolverSettings final {
    double precision;
    double delay;
    double headDelay;  // s
};

template <class Inspector>
bool inspect(Inspector& f, PeriodSolverSettings& x) {
    return f.object(x).fields(f.field("precision", x.precision), f.field("delay", x.delay),
                              f.field("headDelay", x.headDelay).fallback(0.001));
}

class PeriodSolver final : public HubHelper<caf::event_based_actor, PeriodSolverSettings, set_target_info_atom> {
    const double delayTime;
    const Duration mHeadDelay;
    bool mOutpostActive;
    std::atomic_uint mUpdateCnt = 0;

    static constexpr glm::dvec3 tf(const glm::dvec3& ori) {
        return { ori.x, -ori.z, ori.y };
    }

    double getTheta(const glm::dvec3& point) {
        double theta = glm::acos(point.x / glm::sqrt(point.x * point.x + point.z * point.z));
        return point.z < 0 ? glm::two_pi<double>() - theta : theta;
    }

public:
    PeriodSolver(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, delayTime(mConfig.delay), mHeadDelay(doubleCastDuration(mConfig.headDelay)) {}
    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](period_predict_success_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(period_predict_success_atom, TypedIdentifier<PredictedPeriodTarget>);
                ACTOR_EXCEPTION_PROBE();

                //                logInfo("PeriodSolver: received");

                auto data = BlackBoard::instance().get<PredictedPeriodTarget>(key);
                if(!(data.has_value()))
                    return;

                // HubLogger::watch("x", data->position.mVal.x);
                HubLogger::watch("verticalDistance", data->position.mVal.y);
                // HubLogger::watch("z", data->position.mVal.z);
                HubLogger::watch("horizontalDistance", std::sqrt(square(data->position.mVal.z)+square(data->position.mVal.x)));

                glm::dvec3 tfPos = tf(data->position.mVal);
                auto res = solveWithoutAirDrag(tfPos, glm::dvec3{ 0, 0, 0 });
                if(!data->period.has_value()) {
                    sendAllHighPriority(set_target_info_atom_v, mGroupMask, data.value().lastUpdate.time_since_epoch().count(),
                                        std::get<1>(res), std::get<2>(res), false, waitSolver);
                    return;
                }
                double waitTimeDouble = data->period.value() - std::get<0>(res) - delayTime - GlobalSettings::get().latency -
                    GlobalSettings::get().shootDelayTime;
                while(waitTimeDouble < 0)
                    waitTimeDouble += data->period.value();
                auto waitTime = doubleCastDuration(waitTimeDouble);
                logInfo(fmt::format("waitTime {}ms  shootDelay {}ms", static_cast<int>(waitTimeDouble * 1000),
                                    static_cast<int>(GlobalSettings::get().shootDelayTime * 1000)));

                std::thread([this, waitTime, data, res]() {
                    auto t1 = mUpdateCnt.load();

                    Duration firstDelay, secondDelay;
                    if(waitTime > mHeadDelay) {
                        firstDelay = waitTime - mHeadDelay;
                        secondDelay = mHeadDelay;
                    } else {
                        firstDelay = 0s;
                        secondDelay = waitTime;
                    }
                    SynchronizedClock::instance().sleep_for(firstDelay);  // eserve time for turning head
                    if(!mUpdateCnt.compare_exchange_strong(t1, t1))
                        return;

                    sendAllHighPriority(set_target_info_atom_v, mGroupMask,
                                        (data.value().lastUpdate + waitTime - mHeadDelay).time_since_epoch().count(),
                                        std::get<1>(res), std::get<2>(res), false, waitSolver);
                    //                    logInfo("send not shoot");

                    SynchronizedClock::instance().sleep_for(secondDelay);  // ready for shoot
                    if(!mUpdateCnt.compare_exchange_strong(t1, t1))
                        return;

                    sendAllHighPriority(set_target_info_atom_v, mGroupMask,
                                        (data.value().lastUpdate + waitTime).time_since_epoch().count(), std::get<1>(res),
                                        std::get<2>(res), true, waitSolver);  // shoot
                }).detach();
            },
            [this](outpost_detector_control_atom, bool active) {
                ACTOR_PROTOCOL_CHECK(outpost_detector_control_atom, bool);
                if(mOutpostActive ^ active) {  //
                    ++mUpdateCnt;
                }
                mOutpostActive = active;
            },
        };
    }
};

HUB_REGISTER_CLASS(PeriodSolver);
