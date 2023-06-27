#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "SuppressWarningBegin.hpp"

#include <GxIAPI.h>
#include <caf/event_based_actor.hpp>
#include <fmt/core.h>
#include <fmt/format.h>
#include <glm/gtc/matrix_transform.hpp>
#include <magic_enum.hpp>
#include <opencv2/opencv.hpp>
#include <tuple>

#include "SuppressWarningEnd.hpp"

struct HikDriverSettings {};

class HikDriver final : public HubHelper<caf::event_based_actor, HikDriverSettings, image_frame_atom> {
    Identifier mKey;

public:
    HikDriver(caf::actor_config& base, const HubConfig& config) : HubHelper(base, config), mKey{ generateKey(this) } {}

    caf::behavior make_behavior() override {
        return {
            [](start_atom) {}
        };
    }
};
HUB_REGISTER_CLASS(HikDriver);