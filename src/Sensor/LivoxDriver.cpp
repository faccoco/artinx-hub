#ifdef ARTINX_RADAR
#include "BlackBoard.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "RadarInfo.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/core.h>

#include "SuppressWarningEnd.hpp"

#include <cstdint>
#include <livox_def.h>
#include <livox_sdk.h>
#include <string>
#include <type_traits>


struct LivoxDriverSettings final {
    std::string broadercastCode;
};

template <typename Inspector>
bool inspect(Inspector& f, LivoxDriverSettings& x) {
    return f.object(x).fields(f.field("broadercastCode", x.broadercastCode));
}

class LivoxDriver final : public HubHelper<caf::event_based_actor, LivoxDriverSettings, radar_points_atom> {
    Identifier mKey;

    template <typename ValType, char*>
    static void check(ValType status, char* checkName) {
        if constexpr(std::is_same<int32_t, ValType>::value) {
            if(status != static_cast<int32_t>(LivoxStatus::kStatusSuccess)) {
                const auto error = "Livox error in " + std::string(checkName) + " code: " + std::to_string(status);
                logError(error);
            }
        } else if constexpr(std::is_same<bool, ValType>::value) {
            if(!status) {
                const auto error = "Livox Error happened in " + std::string(checkName);
                logError(error);
            }
        } else {
            // static_assert(false, "123");
        }
    }

public:
    LivoxDriver(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}

    ~LivoxDriver() {}

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); }, [](update_radar_atom, Identifier key) {} };
    }
};

HUB_REGISTER_CLASS(LivoxDriver);
#endif
