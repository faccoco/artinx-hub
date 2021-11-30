#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedTarget.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include "ClassifiedNum.hpp"
#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/random.hpp>
#include <queue>

struct sentryStrategySettings final {
    double distanceThreshold;
};


constexpr int32_t engineerId = 2;

template <class Inspector>
bool inspect(Inspector& f, sentryStrategySettings& x) {
    return f.object(x).fields(f.field("distanceThreshold", x.distanceThreshold));
}

class sentryStrategy final : public HubHelper<caf::event_based_actor, sentryStrategySettings, set_target_atom> {
    Identifier mKey;

public:
    sentryStrategy(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(sentryStrategy).hash_code() } {}
    caf::behavior make_behavior() override {
        return {
            [this](start_atom) {}, [&](detect_available_atom, Identifier key) {
                const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                SelectedTarget selected;
                selected.lastUpdate = data.lastUpdate;

                auto minDistance = std::numeric_limits<double>::max();
                for(auto& target : data.targets) {
                    const auto distance = glm::length(target.center.raw());
                    if(distance > mConfig.distanceThreshold && distance < minDistance) {
                        if(target.id != engineerId) {
                            selected.center = target.center;
                            minDistance = distance;
                        }
                    }
                }
                BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected);
                sendAll(set_target_atom_v, mKey);
            }
        };
    }
};

HUB_REGISTER_CLASS(sentryStrategy);

