#ifdef ARTINX_RADAR
#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/core.h>
#include <fmt/format.h>
#include <glm/gtc/matrix_transform.hpp>
//#include <livox_sdk.h>
#include <magic_enum.hpp>
#include <opencv2/opencv.hpp>
#include <tuple>

#include "SuppressWarningEnd.hpp"
struct LivoxHorizonSettings final {};
template <typename Inspector>
bool inspect(Inspector& f, LivoxHorizonSettings& x) {
    return f.object(x).fields();
}

class LivoxHorizon final : public HubHelper<caf::event_based_actor, LivoxHorizonSettings, void> {
    Identifier mKey;

public:
    LivoxHorizon(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); } };
    }
};

HUB_REGISTER_CLASS(LivoxHorizon);
#endif
