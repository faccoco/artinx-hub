#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedCar.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>

struct ArmorDetectorSettings final {};
template <class Inspector>
bool inspect(Inspector& f, ArmorDetectorSettings& x) {
    return f.object(x).fields();
}

class ArmorDetector final : public HubHelper<caf::event_based_actor, ArmorDetectorSettings, armor_detect_available_atom> {
    Identifier mKey;

    std::vector<PairedLight> solve(const cv::Mat& image) {
        // Implement here
        // Color classification is not required
        return {};
    }

public:
    ArmorDetector(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(ArmorDetector).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](car_detect_available_atom, Identifier key) {
                     const auto data = BlackBoard::instance().get<DetectedCarArray>(key).value();

                     DetectedArmorArray res;
                     res.lastUpdate = data.frame.lastUpdate;
                     res.cameraInfo = data.frame.info;

                     for(auto& roi : data.cars) {
                         auto armors = solve(data.frame.frame(roi));
                         res.armors.push_back({ roi, 0, std::move(armors) });  // TODO: id
                     }

                     BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(armor_detect_available_atom_v, mKey);
                 } };
    }
};

HUB_REGISTER_CLASS(ArmorDetector);
