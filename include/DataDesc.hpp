#pragma once
#include "Constants.hpp"
#include <caf/allowed_unsafe_message_type.hpp>
#include <caf/is_error_code_enum.hpp>
#include <caf/type_id.hpp>
#include <cstdint>

struct GlobalSettings final {
    double gForce;
    double dragCoefficient;
    bool bullet42mm;

    double bulletRadius() const noexcept {
        return bullet42mm ? radiusOf42mm : radiusOf17mm;
    }
    double bulletMass() const noexcept {
        return bullet42mm ? massOf42mm : massOf17mm;
    }
};

template <class Inspector>
bool inspect(Inspector& f, GlobalSettings& x) {
    return f.object(x).fields(f.field("gForce", x.gForce), f.field("dragCoefficient", x.dragCoefficient),
                              f.field("bullet42mm", x.bullet42mm));
}

struct Identifier final {
    uint64_t val;
};

CAF_BEGIN_TYPE_ID_BLOCK(ArtinxHub, caf::first_custom_type_id);

CAF_ADD_ATOM(ArtinxHub, start_atom);
CAF_ADD_ATOM(ArtinxHub, shoot_atom);
CAF_ADD_ATOM(ArtinxHub, detect_available_atom);
CAF_ADD_ATOM(ArtinxHub, set_target_atom);
CAF_ADD_ATOM(ArtinxHub, set_target_posture_atom);
CAF_ADD_ATOM(ArtinxHub, update_posture_atom);
CAF_ADD_ATOM(ArtinxHub, update_head_atom);
CAF_ADD_ATOM(ArtinxHub, simulator_step_atom);
CAF_ADD_ATOM(ArtinxHub, timer_atom);
CAF_ADD_ATOM(ArtinxHub, image_frame_atom);
CAF_ADD_ATOM(ArtinxHub, car_detect_available_atom);
CAF_ADD_ATOM(ArtinxHub, armor_detect_available_atom);
CAF_ADD_ATOM(ArtinxHub, energy_detect_available_atom);


CAF_ADD_TYPE_ID(ArtinxHub, (Identifier));

CAF_END_TYPE_ID_BLOCK(ArtinxHub);

CAF_ALLOW_UNSAFE_MESSAGE_TYPE(Identifier);

