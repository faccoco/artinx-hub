#include "Timer.hpp"
#include "DataDesc.hpp"
#include <caf/scoped_actor.hpp>

Timer& Timer::instance() {
    static Timer inst;
    return inst;
}

Timer::Timer() {
    mThread = std::thread{ [&] {
        using namespace std::chrono_literals;
        while(mRunFlag) {
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

            const caf::scoped_actor caller{ *mSystem };
            caller->send(actor, timer_atom_v);
        }
    } };
}

void Timer::bindSystem(caf::actor_system& system) {
    mSystem = &system;
}

Timer::~Timer() {
    mRunFlag = false;
    mThread.join();
}

void Timer::addTimer(caf::actor actor, Duration period) {
    std::lock_guard<std::mutex> guard{ mMutex };
}

TimePoint SynchronizedClock::now() {
    return Clock::now();
}
