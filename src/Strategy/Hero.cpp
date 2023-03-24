#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedTarget.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"

class HeroStrategy final : public HubHelper<caf::event_based_actor, void, set_target_atom, set_period_outpost_atom> {
    Identifier mKey;
    bool mOutpostActive = false, mOutpostInited = false;

public:
    HeroStrategy(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](detect_available_atom, GroupMask, Identifier key) {
                ACTOR_PROTOCOL_CHECK(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
                const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                SelectedTarget selected;
                selected.lastUpdate = data.lastUpdate;
                selected.tfRobot2Gun = data.tfRobot2Gun;

                auto minDistance = std::numeric_limits<double>::max();
                for(auto& target : data.targets) {
                    if(const auto distance = glm::length(target.center.mVal); minDistance > distance) {
                        selected.selected = target;
                        minDistance = distance;
                    }
                }
                if(mOutpostActive) {
                    sendAll(set_period_outpost_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected),
                            !mOutpostInited);
                    if(selected.selected.has_value() && selected.tfRobot2Gun.has_value() && !mOutpostInited)
                        mOutpostInited = true;
                } else {
                    sendAll(set_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
                }
            },
            [this](outpost_detector_control_atom, bool active) {
                ACTOR_PROTOCOL_CHECK(outpost_detector_control_atom, bool);
                if(active && !mOutpostActive)
                    mOutpostInited = false;
                mOutpostActive = active;
            },
        };
    }
};

HUB_REGISTER_CLASS(HeroStrategy);
