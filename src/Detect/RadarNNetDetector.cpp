#ifdef ARTINX_RADAR
#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"

#include "SuppressWarningBegin.hpp"

#include <NvInfer.h>
#include <caf/event_based_actor.hpp>

#include "SuppressWarningEnd.hpp"

#include <caf/actor_config.hpp>
#include <caf/event_based_actor.hpp>
struct RadarNNetDetectorSettings final {
    std::string modulePath;
};

template <typename Inspector>
bool inspect(Inspector& f, RadarNNetDetectorSettings& x) {
    return f.object(x).fields(f.field("modulePath", x.modulePath));
}

class RadarNNetDetector final
    : public HubHelper<caf::event_based_actor, RadarNNetDetectorSettings, bots_locate_succeed_atom, image_frame_atom> {
    Identifier mKey;

public:
    RadarNNetDetector(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); }, [](bots_locate_request_atom, Identifier key) {} };
    }
};

HUB_REGISTER_CLASS(RadarNNetDetector);
#endif
