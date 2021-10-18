#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "DetectedCar.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>

struct CarDetectorSettings final {};
template <class Inspector>
bool inspect(Inspector& f, CarDetectorSettings& x) {
    return f.object(x).fields();
}

class CarDetector final : public HubHelper<caf::event_based_actor, CarDetectorSettings, armor_detect_available_atom> {
    Identifier mKey;

public:
    CarDetector(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(CarDetector).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](image_frame_atom, Identifier key) {
                     DetectedCarArray res;
                     res.frame = BlackBoard::instance().get<CameraFrame>(key).value();

                     // TODO: Color detection
                     // Implement here

                     BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(armor_detect_available_atom_v, mKey);
                 } };
    }
};

HUB_REGISTER_CLASS(CarDetector);
