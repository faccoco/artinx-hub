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

struct SimpleStrategySettings final {
    std::string periodPredictType;
};

template <class Inspector>
bool inspect(Inspector& f, SimpleStrategySettings& x) {
    return f.object(x).fields(f.field("periodPredictType", x.periodPredictType).fallback("outpost"));
}

class SimpleStrategy final : public HubHelper<caf::event_based_actor, SimpleStrategySettings, set_target_atom,
                                              set_period_target_atom, set_period_outpost_atom> {
    Identifier mKey;
    std::function<void(SelectedTarget)> mSendPeriodFunc;
    bool mPeriodActive = false, mPeriodInited = false;

public:
    SimpleStrategy(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        if(mConfig.periodPredictType == "outpost")
            mSendPeriodFunc = [this](const SelectedTarget& selected) {
                sendAll(set_period_outpost_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected),
                        !mPeriodInited);
            };
        else if(mConfig.periodPredictType == "car")
            mSendPeriodFunc = [this](const SelectedTarget& selected) {
                sendAll(set_period_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected),
                        !mPeriodInited);
            };
        else {
            logInfo("HeroStrategy wrong periodPredictType using fallback \"outpost\"");
            mSendPeriodFunc = [this](const SelectedTarget& selected) {
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
                selected.targets = data.targets;

                double minDisToImgCenter = std::numeric_limits<double>::max();
                for(const auto& target : selected.targets) {
                    if(target.distToImgCenter < minDisToImgCenter) {
                        selected.selected = target;
                        minDisToImgCenter = target.distToImgCenter;
                    }
                }
                if(mPeriodActive) {
                    mSendPeriodFunc(selected);
                    if(selected.selected.has_value() && !mPeriodInited)
                        mPeriodInited = true;
                } else {
                    sendAll(set_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
                }
                if(selected.selected.has_value())
                    HubLogger::watch("armor type", magic_enum::enum_name(selected.selected->type));
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

HUB_REGISTER_CLASS(SimpleStrategy);
