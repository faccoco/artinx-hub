#include "DataDesc.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <csignal>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>

#include "SuppressWarningEnd.hpp"

class BlockedActor final : public HubHelper<caf::event_based_actor, void, payload_atom> {
    inline static bool mFailed = false;
    static size_t restartCount;

public:
    BlockedActor(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](payload_atom, int32_t, int32_t) {
                     ACTOR_PROTOCOL_CHECK(payload_atom, int32_t, int32_t);
                     if(mFailed)
                         sendAll(payload_atom_v, 0, 0);
                     else {
                         mFailed = true;
                         while(true)
                             ;
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(BlockedActor);

class ExceptionKilled final : public HubHelper<caf::event_based_actor, void, payload_atom> {
    inline static bool mFailed = false;
    static size_t restartCount;

public:
    ExceptionKilled(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](payload_atom, int32_t, int32_t) {
                     logError(fmt::format("exception killed node restart count: {}", ++restartCount));
                     if(mFailed)
                         sendAll(payload_atom_v, 0, 0);
                     else {
                         //                         mFailed = true;
                         throw std::runtime_error("Exception Down Test");
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(ExceptionKilled);

class SignalKilled final : public HubHelper<caf::event_based_actor, void, payload_atom> {
    inline static bool mFailed = false;
    static size_t restartCount;

public:
    SignalKilled(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](payload_atom, int32_t, int32_t) {
                     logInfo(fmt::format("signal killed node restart count: {}", ++restartCount));
                     if(mFailed)
                         sendAll(payload_atom_v, 0, 0);
                     else {
                         raise(SIGEV_SIGNAL);
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(SignalKilled);

class DaemonTester final : public HubHelper<caf::event_based_actor, void, payload_atom> {
    bool mStarted = false;
    static size_t restartCount;

public:
    DaemonTester(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {
        Timer::instance().addTimer(address(), 5ms);
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                    mStarted = true;
                },
                 [this](timer_atom) {
                     ACTOR_PROTOCOL_CHECK(timer_atom);
                     if(!mStarted)
                         return;
                     if(++restartCount >= 5000)
                         terminateSystem(*this, true);
                     else {
                         //                         logInfo("=== sending message to actor ===\n");
                         sendAll(payload_atom_v, 0, 0);
                     }
                 } };
    }
};

size_t BlockedActor::restartCount = 0;
size_t ExceptionKilled::restartCount = 0;
size_t SignalKilled::restartCount = 0;
size_t DaemonTester::restartCount = 0;
HUB_REGISTER_CLASS(DaemonTester);
