#pragma once
#include <caf/actor_addr.hpp>
#include <chrono>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Duration = Clock::duration;

class SynchronizedClock final {
    std::optional<TimePoint> mSimulationTime;

public:
    void setSimulationTime(TimePoint tp);
    TimePoint now() const;
    static SynchronizedClock& instance();
};

class Timer final {
    caf::actor_system* mSystem = nullptr;
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
    Timer();
    ~Timer();

    Timer(const Timer& rhs) = delete;
    Timer(Timer&& rhs) = delete;
    Timer& operator=(const Timer& rhs) = delete;
    Timer& operator=(Timer&& rhs) = delete;

    void bindSystem(caf::actor_system& system);
    void addTimer(caf::actor_addr actor, Duration period);
    static Timer& instance();
};
