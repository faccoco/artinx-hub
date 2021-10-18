#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>

struct UndistortSettings final {};

template <class Inspector>
bool inspect(Inspector& f, UndistortSettings& x) {
    return f.object(x).fields();
}

class Undistort final : public HubHelper<caf::event_based_actor, UndistortSettings, image_frame_atom> {
    Identifier mKey;

public:
    Undistort(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(Undistort).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](image_frame_atom, Identifier key) {
                     auto res = BlackBoard::instance().get<CameraFrame>(key).value();

                     // Implement here

                     BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(image_frame_atom_v, mKey);
                 } };
    }
};

HUB_REGISTER_CLASS(Undistort);

struct UndistortCalibratorSettings final {};

template <class Inspector>
bool inspect(Inspector& f, UndistortCalibratorSettings& x) {
    return f.object(x).fields();
}

class UndistortCalibrator final : public HubHelper<caf::event_based_actor, UndistortCalibratorSettings, image_frame_atom> {
    Identifier mKey;

public:
    UndistortCalibrator(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(UndistortCalibrator).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](image_frame_atom, Identifier key) {
                     const auto input = BlackBoard::instance().get<CameraFrame>(key).value();

                     // Implement here
                     CameraFrame res = input;

                     // For debugging
                     BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(image_frame_atom_v, mKey);
                 } };
    }
};

HUB_REGISTER_CLASS(UndistortCalibrator);
