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
    enum class ScheduleState : uint8_t { empty, send, notSend };
    std::vector<ScheduleState> mSendSchedule;
    std::mutex mScheduleMutex;

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

                logInfo("PeriodSolver: received");

                auto data = BlackBoard::instance().get<PredictedPeriodTarget>(key);
                if(!(data.has_value()))
                    return;

                HubLogger::watch("x", data->position.mVal.x);
                HubLogger::watch("y", data->position.mVal.y);
                HubLogger::watch("z", data->position.mVal.z);

                glm::dvec3 tfPos = tf(data->position.mVal);
                auto res = solveWithoutAirDrag(tfPos, glm::dvec3{ 0, 0, 0 });
                if(!data->period.has_value()) {
                    sendAllHighPriority(set_target_info_atom_v, mGroupMask, data.value().lastUpdate.time_since_epoch().count(),
                                        std::get<1>(res), std::get<2>(res), false, solverType_period);
                    return;
                }
                double waitTime = data->period.value() - std::get<0>(res) - delayTime;
                while(waitTime < 0)
                    waitTime += data->period.value();
                std::thread([this, waitTime, data, res]() {
                    ScheduleState* sc = NULL;
                    std::unique_lock lock(mScheduleMutex);
                    for(auto schedule : mSendSchedule) {
                        if(schedule == ScheduleState::empty) {
                            sc = &schedule;
                            schedule = ScheduleState::send;
                            break;
                        }
                    }
                    if(sc == NULL) {
                        mSendSchedule.emplace_back(ScheduleState::send);
                        sc = &mSendSchedule.back();
                        logInfo(fmt::format("mSendSchedule emplaced. len = {}", mSendSchedule.size()));
                    }
                    lock.unlock();

                    SynchronizedClock::instance().sleep_for(doubleCastDuration(waitTime) - mHeadDelay);

                    lock.lock();
                    if(*sc == ScheduleState::notSend) {
                        logInfo("schedule interrupt");
                        *sc = ScheduleState::empty;
                        return;
                    }
                    lock.unlock();

                    sendAllHighPriority(set_target_info_atom_v, mGroupMask, data.value().lastUpdate.time_since_epoch().count(),
                                        std::get<1>(res), std::get<2>(res), false, solverType_period);
                    logInfo("send not shoot");

                    SynchronizedClock::instance().sleep_for(mHeadDelay);

                    lock.lock();
                    if(*sc == ScheduleState::notSend) {
                        logInfo("schedule interrupt");
                        *sc = ScheduleState::empty;
                        return;
                    }
                    lock.unlock();

                    sendAllHighPriority(set_target_info_atom_v, mGroupMask, data.value().lastUpdate.time_since_epoch().count(),
                                        std::get<1>(res), std::get<2>(res), true, solverType_period);
                    logInfo("send shoot");
                    // logInfo(fmt::format("solver: x: {} y: {} z: {}", data->position.mVal.x, data->position.mVal.y,
                    //                     data->position.mVal.z));
                    // logInfo(fmt::format("solver: yaw: {} pitch: {}", 270 - glm::degrees(std::get<1>(res)),
                    //                     glm::degrees(std::get<2>(res))));
                    // logInfo(fmt::format("solver: theta: {}", glm::degrees(getTheta(data->position.mVal))));
                }).detach();
            },
            [this](outpost_detector_control_atom, bool active) {
                ACTOR_PROTOCOL_CHECK(outpost_detector_control_atom, bool);
                if(mOutpostActive && !active) {
                    std::lock_guard lock(mScheduleMutex);
                    for(auto schedule : mSendSchedule)
                        if(schedule == ScheduleState::send)
                            schedule = ScheduleState::notSend;
                }
                mOutpostActive = active;
            },
        };
    }
};

HUB_REGISTER_CLASS(PeriodSolver);
