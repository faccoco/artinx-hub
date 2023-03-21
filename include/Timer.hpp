#pragma once
#include "DataDesc.hpp"
#include "SuppressWarningBegin.hpp"

#include <caf/actor_addr.hpp>

#include "SuppressWarningEnd.hpp"
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>

using TimePoint = Clock::time_point;
using Duration = Clock::duration;

constexpr double durationCastDouble(const Duration& d) {
    return double(d.count()) / Duration::period::den * Duration::period::num;
}

constexpr Duration doubleCastDuration(const double d) {
    return Duration(Duration::rep(d * Duration::period::den / Duration::period::num));
}

class SynchronizedClock final {
    std::mutex mMutex;
    std::optional<TimePoint> mSimulationTime;
    std::optional<Duration> mSimulationTimeStep;
    std::optional<Duration> mSimulationTimeHalfStep;
    struct TimerInfo {
        std::condition_variable* cv;
        TimePoint ddl;
        bool operator<(const TimerInfo& rhs) const {
            return ddl > rhs.ddl;
        }
    };
    static std::priority_queue<TimerInfo> mSleepForQueue;

public:
    void setSimulationTime(TimePoint tp);
    void setSimulationTimeStep(Duration dt);
    [[nodiscard]] TimePoint now() const;
    void sleep_for(const Duration& d);
    static SynchronizedClock& instance();
};

class Timer final {
    std::mutex mMutex;
    struct TimerInfo final {
        caf::actor_addr address;
        TimePoint deadline;
        Duration duration;
        bool operator<(const TimerInfo& rhs) const noexcept {
            return deadline > rhs.deadline;
        }
    };
    std::priority_queue<TimerInfo> mTimers;
    std::thread mThread;

public:
    Timer() = default;
    ~Timer() = default;
    Timer(const Timer& rhs) = delete;
    Timer(Timer&& rhs) = delete;
    Timer& operator=(const Timer& rhs) = delete;
    Timer& operator=(Timer&& rhs) = delete;

    void bindSystem(caf::actor_system& system);
    void stop();
    void addTimer(caf::actor_addr actor, Duration period);
    static Timer& instance();
};
