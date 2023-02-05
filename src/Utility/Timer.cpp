#include "Timer.hpp"
#include "DataDesc.hpp"
#include "Utility.hpp"
#include <utility>

#include "SuppressWarningBegin.hpp"

#include <caf/scoped_actor.hpp>

#include "SuppressWarningEnd.hpp"

Timer& Timer::instance() {
    static Timer inst;
    return inst;
}

void Timer::bindSystem(caf::actor_system& system) {
    mThread = std::thread{ [&] {
        using namespace std::chrono_literals;
        while(globalStatus == RunStatus::running) {
            if(mTimers.empty()) {
                std::this_thread::sleep_for(10ms);
                continue;
            }

            std::unique_lock<std::mutex> guard{ mMutex, std::try_to_lock };
            if(!guard.owns_lock()) {
                std::this_thread::yield();
                continue;
            }

            const auto [actor, deadline, duration] = mTimers.top();
            mTimers.pop();
            mTimers.push({ actor, deadline + duration, duration });

            guard.unlock();

            std::this_thread::sleep_until(deadline);

            const caf::scoped_actor caller{ system };
            caller->send(caf::actor_cast<caf::actor>(actor), timer_atom_v);
        }
    } };
}

void Timer::stop() {
    if(mThread.joinable())
        mThread.join();
}

void Timer::addTimer(caf::actor_addr actor, Duration period) {
    std::lock_guard<std::mutex> guard{ mMutex };
    mTimers.push({ std::move(actor), Clock::now() + period, period });
}

std::priority_queue<SynchronizedClock::TimerInfo> SynchronizedClock::mSleepForQueue;

TimePoint SynchronizedClock::now() const {
    if(mSimulationTime)
        return mSimulationTime.value();

    return Clock::now();
}

void SynchronizedClock::sleep_for(const Duration& rawD) {
    std::unique_lock lock(mMutex);
    if(mSimulationTime.has_value() && mSimulationTimeStep.has_value()) {
        lock.unlock();
        Duration remainder = rawD % mSimulationTimeStep.value();
        Duration d = (remainder >= mSimulationTimeHalfStep ? rawD - remainder + mSimulationTimeStep.value() : rawD - remainder);
        std::condition_variable cv;
        lock.lock();
        mSleepForQueue.push(TimerInfo{ &cv, now() + d });
        cv.wait(lock);
    } else
        std::this_thread::sleep_for(rawD);
}

SynchronizedClock& SynchronizedClock::instance() {
    static SynchronizedClock clock;
    return clock;
}

void SynchronizedClock::setSimulationTime(TimePoint tp) {
    std::lock_guard lock(mMutex);
    mSimulationTime = tp;
    while(!mSleepForQueue.empty() && (mSleepForQueue.top().ddl - mSimulationTime.value()) < mSimulationTimeHalfStep) {
        mSleepForQueue.top().cv->notify_one();
        mSleepForQueue.pop();
    }
}

void SynchronizedClock::setSimulationTimeStep(Duration dt) {
    std::lock_guard lock(mMutex);
    mSimulationTimeStep = dt;
    mSimulationTimeHalfStep = dt / 2;
}
