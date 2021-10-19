#pragma once
#include <caf/actor.hpp>
#include <chrono>
#include <mutex>
#include <queue>
#include <thread>

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Duration = Clock::duration;

class SynchronizedClock final {
public:
    static TimePoint now();
};

class Timer final {
    caf::actor_system* mSystem = nullptr;
    std::mutex mMutex;
    struct TimerInfo final {
        caf::actor address;
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
    void addTimer(caf::actor actor, Duration period);
    static Timer& instance();
};
