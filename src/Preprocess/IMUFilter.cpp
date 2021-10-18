#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>

struct IMUFilterSettings final {};

template <class Inspector>
bool inspect(Inspector& f, IMUFilterSettings& x) {
    return f.object(x).fields();
}

class IMUFilter final : public HubHelper<caf::event_based_actor, IMUFilterSettings, update_posture_atom> {
    Identifier mKey;

public:
    IMUFilter(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(IMUFilter).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](update_posture_atom, Identifier key) {
                     // Implement here

                     //BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(update_posture_atom_v, mKey);
                 } };
    }
};

HUB_REGISTER_CLASS(IMUFilter);