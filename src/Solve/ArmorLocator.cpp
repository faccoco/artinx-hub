#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedTarget.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>

struct ArmorLocatorSettings final {};

template <class Inspector>
bool inspect(Inspector& f, ArmorLocatorSettings& x) {
    return f.object(x).fields();
}

class ArmorLocator final : public HubHelper<caf::event_based_actor, ArmorLocatorSettings, detect_available_atom> {
    Identifier mKey, mHeadKey{};

    Point<UnitType::Distance, FrameOfReference::Camera> solve(const CameraInfo& info, const PairedLight& armor) const {
        // Implement here
        return {};
    }

public:
    ArmorLocator(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(ArmorLocator).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](armor_detect_available_atom, Identifier key) {
                     const auto data = BlackBoard::instance().get<DetectedArmorArray>(key).value();
                     DetectedTargetArray res;
                     res.lastUpdate = data.lastUpdate;

                     Transform<FrameOfReference::Gun, FrameOfReference::Camera, true> transform;
                     if(data.cameraInfo.transform.index() == 0) {
                         transform = std::get<0>(data.cameraInfo.transform);
                     } else {
                         const auto& trans = std::get<1>(data.cameraInfo.transform);
                         const auto headTrans = BlackBoard::instance().get<HeadInfo>(mHeadKey).value().transform;
                         transform =
                             static_cast<Transform<FrameOfReference::Gun, FrameOfReference::Robot, true>>(headTrans) * trans;
                     }

                     for(auto& cars : data.armors) {
                         for(auto& armor : cars.armors) {
                             auto armorLight = armor;
                             armorLight.r1.center += cv::Point2f{ cars.roi.tl() };
                             armorLight.r2.center += cv::Point2f{ cars.roi.tl() };

                             const auto point = solve(data.cameraInfo, armorLight);

                             // TODO: projected area
                             res.targets.push_back({ transform(point), 0.0, cars.id });
                         }
                     }

                     BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(detect_available_atom_v, mKey);
                 },
                 [&](update_head_atom, Identifier key) { mHeadKey = key; } };
    }
};

HUB_REGISTER_CLASS(ArmorLocator);
