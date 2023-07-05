#pragma once
#include "Constants.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/allowed_unsafe_message_type.hpp>
#include <caf/is_error_code_enum.hpp>
#include <caf/type_id.hpp>
#include <cstddef>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"

#include <cstdint>

using Clock = std::chrono::steady_clock;
static_assert(std::is_same_v<Clock::period, std::nano>);

enum class Color { Blue, Red, Purple, Negative };

struct GlobalSettings final {
    double gForce;
    double dragCoefficient;
    double airDensity;
    bool bullet42mm;

    double latency = 0.0;

    bool isRed;
    double bulletSpeed;
    double shootDelayTime = 0.f;
    bool started = false;

    [[nodiscard]] double bulletRadius() const noexcept {
        return bullet42mm ? radiusOf42mm : radiusOf17mm;
    }

    [[nodiscard]] double bulletMass() const noexcept {
        return bullet42mm ? massOf42mm : massOf17mm;
    }

    [[nodiscard]] Color getColor() const noexcept {
        return isRed ? Color::Red : Color::Blue;
    }

    void setColor(Color color) noexcept {
        isRed = (color != Color::Blue);
    }

    static GlobalSettings& get() {
        static GlobalSettings settings;
        return settings;
    }
};

template <class Inspector>
bool inspect(Inspector& f, GlobalSettings& x) {
    return f.object(x).fields(f.field("gForce", x.gForce), f.field("dragCoefficient", x.dragCoefficient).fallback(0),
                              f.field("airDensity", x.airDensity).fallback(0), f.field("bullet42mm", x.bullet42mm),
                              f.field("defaultBulletSpeed", x.bulletSpeed).fallback(9.00),
                              f.field("isRed", x.isRed).fallback(true));
}

struct Identifier {
    uint64_t val;
};

template <size_t lhs, size_t rhs>
struct StaticCompare {
    const static bool result = lhs == rhs;
};

template <bool B, typename T, typename... TL>
struct StaticIdentify {};

template <typename T, typename... TL>
struct StaticIdentify<true, T, TL...> {
    using Payload = T;
};

template <typename T, typename... TL>
struct StaticIdentify<false, T, TL...> {
    using Payload = std::tuple<T, TL...>;
};

template <typename T, typename... TL>
struct TypedIdentifier final : Identifier {
    using Payload = typename StaticIdentify<StaticCompare<sizeof...(TL), 0>::result, T, TL...>::Payload;
};

CAF_BEGIN_TYPE_ID_BLOCK(ArtinxHub, caf::first_custom_type_id);

CAF_ADD_ATOM(ArtinxHub, start_atom);
CAF_ADD_ATOM(ArtinxHub, detect_available_atom);
CAF_ADD_ATOM(ArtinxHub, set_target_atom);
CAF_ADD_ATOM(ArtinxHub, set_period_target_atom);
CAF_ADD_ATOM(ArtinxHub, set_period_outpost_atom);
CAF_ADD_ATOM(ArtinxHub, set_target_info_atom);
CAF_ADD_ATOM(ArtinxHub, sync_position_atom);
CAF_ADD_ATOM(ArtinxHub, update_posture_atom);
CAF_ADD_ATOM(ArtinxHub, update_head_atom);
CAF_ADD_ATOM(ArtinxHub, update_radar_atom);
CAF_ADD_ATOM(ArtinxHub, simulator_step_atom);
CAF_ADD_ATOM(ArtinxHub, timer_atom);
CAF_ADD_ATOM(ArtinxHub, image_frame_atom);
CAF_ADD_ATOM(ArtinxHub, radar_points_atom);
CAF_ADD_ATOM(ArtinxHub, armor_detect_available_atom);
CAF_ADD_ATOM(ArtinxHub, car_detect_available_atom);
CAF_ADD_ATOM(ArtinxHub, energy_detect_available_atom);
CAF_ADD_ATOM(ArtinxHub, predict_success_atom);
CAF_ADD_ATOM(ArtinxHub, car_predict_atom);
CAF_ADD_ATOM(ArtinxHub, period_predict_success_atom);
CAF_ADD_ATOM(ArtinxHub, radar_locate_request_atom);
CAF_ADD_ATOM(ArtinxHub, bots_locate_request_atom);
CAF_ADD_ATOM(ArtinxHub, num_classify_request_atom);
CAF_ADD_ATOM(ArtinxHub, monitor_request_atom);
CAF_ADD_ATOM(ArtinxHub, monitor_response_atom);
CAF_ADD_ATOM(ArtinxHub, payload_atom);
CAF_ADD_ATOM(ArtinxHub, energy_detector_control_atom);
CAF_ADD_ATOM(ArtinxHub, hero_strategy_control_atom);

CAF_ADD_TYPE_ID(ArtinxHub, (Identifier));

CAF_END_TYPE_ID_BLOCK(ArtinxHub);

CAF_ALLOW_UNSAFE_MESSAGE_TYPE(Identifier);

using GroupMask = uint32_t;
using SolverType = uint8_t;  // 0 normal track; 1: wait for target
static constexpr SolverType normalSolver = 0, waitSolver = 1;

template <typename... T>
struct __ImplActorProtocol final {
    static constexpr bool check() noexcept {
        return false;
    }
};

#define ACTOR_PROTOCOL_DEFINE(...)                              \
    template <>                                                 \
    struct __ImplActorProtocol<__VA_ARGS__> final {             \
        static constexpr bool check() noexcept { return true; } \
    }

template <typename... Args>
constexpr bool __impl_actor_protocol_call() noexcept {
    return __ImplActorProtocol<Args...>::check();
}

#define ACTOR_PROTOCOL_CHECK(...) static_assert(__impl_actor_protocol_call<__VA_ARGS__>(), "Mismatched protocol")

ACTOR_PROTOCOL_DEFINE(start_atom);
ACTOR_PROTOCOL_DEFINE(timer_atom);
ACTOR_PROTOCOL_DEFINE(monitor_response_atom);
ACTOR_PROTOCOL_DEFINE(payload_atom, int32_t, int32_t);
