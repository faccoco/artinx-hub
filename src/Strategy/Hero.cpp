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

struct HeroStrategySettings final {
    std::string periodPredictType;
};

template <class Inspector>
bool inspect(Inspector& f, HeroStrategySettings& x) {
    return f.object(x).fields(f.field("periodPredictType", x.periodPredictType).fallback("outpost"));
}

class HeroStrategy final : public HubHelper<caf::event_based_actor, HeroStrategySettings, set_target_atom, set_period_target_atom,
                                            set_period_outpost_atom> {
    Identifier mKey;
    std::function<void(SelectedTarget)> mSendPeriodFunc;
    bool mPeriodActive = false, mPeriodInited = false;

public:
    HeroStrategy(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        if(mConfig.periodPredictType == "outpost")
            mSendPeriodFunc = [this](SelectedTarget selected) {
                sendAll(set_period_outpost_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected),
                        !mPeriodInited);
            };
        else if(mConfig.periodPredictType == "car")
            mSendPeriodFunc = [this](SelectedTarget selected) {
                sendAll(set_period_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected),
                        !mPeriodInited);
            };
        else {
            logInfo("HeroStrategy wrong periodPredictType using fallback \"outpost\"");
            mSendPeriodFunc = [this](SelectedTarget selected) {
                sendAll(set_period_outpost_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected),
                        !mPeriodInited);
            };
        }
    }
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
                    if(const auto distance = glm::length(target.center.mVal);
                       ((target.type == selected.selected->type) && (minDistance > distance)) ||
                       ((target.type == ArmorType::Large) && (selected.selected->type == ArmorType::Small))) {
                        selected.selected = target;
                        minDistance = distance;
                    }
                }
                if(mPeriodActive) {
                    mSendPeriodFunc(selected);
                    if(selected.selected.has_value() && selected.tfRobot2Gun.has_value() && !mPeriodInited)
                        mPeriodInited = true;
                } else {
                    sendAll(set_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
                }
            },
            [this](outpost_detector_control_atom, bool active) {
                ACTOR_PROTOCOL_CHECK(outpost_detector_control_atom, bool);
                if(active && !mPeriodActive)
                    mPeriodInited = false;
                mPeriodActive = active;
            },
        };
    }
};

HUB_REGISTER_CLASS(HeroStrategy);
