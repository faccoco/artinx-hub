#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>

struct AngleSolverSettings final {};

template <class Inspector>
bool inspect(Inspector& f, AngleSolverSettings& x) {
    return f.object(x).fields();
}

class AngleSolver final : public HubHelper<caf::event_based_actor, AngleSolverSettings, shoot_atom, set_target_posture_atom> {
    Identifier mKey, mIMUKey, mHeadKey;

public:
    AngleSolver(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(AngleSolver).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [this](set_target_atom, Identifier key) {
                     const auto data = BlackBoard::instance().get<SelectedTarget>(key);

                     sendAll(set_target_posture_atom_v, 0.0, 0.0);
                     sendAll(shoot_atom_v, true);
                 },
                 [this](update_head_atom, Identifier key) { mHeadKey = key; },
                 [this](update_posture_atom, Identifier key) { mIMUKey = key; } };
    }
};

HUB_REGISTER_CLASS(AngleSolver);
