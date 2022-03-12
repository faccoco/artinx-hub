#include "DataDesc.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <caf/event_based_actor.hpp>
#include <csignal>

class BlockedActor final : public HubHelper<caf::event_based_actor, void, payload_atom> {
    inline static bool mFailed = false;

public:
    BlockedActor(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](payload_atom) {
                     if(mFailed)
                         sendAll(payload_atom_v);
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

public:
    ExceptionKilled(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](payload_atom) {
                     if(mFailed)
                         sendAll(payload_atom_v);
                     else {
                         mFailed = true;
                         throw std::runtime_error("emulated exception");
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(ExceptionKilled);

class SignalKilled final : public HubHelper<caf::event_based_actor, void, payload_atom> {
    inline static bool mFailed = false;

public:
    SignalKilled(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](payload_atom) {
                     if(mFailed)
                         sendAll(payload_atom_v);
                     else {
                         mFailed = true;
                         raise(SIGSEGV);
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(SignalKilled);

class DaemonTester final : public HubHelper<caf::event_based_actor, void, payload_atom> {
public:
    DaemonTester(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) { sendAll(payload_atom_v); }, [&](payload_atom) { terminateSystem(*this, true); } };
    }
};

HUB_REGISTER_CLASS(DaemonTester);
