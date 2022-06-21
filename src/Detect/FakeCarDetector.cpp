#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "DetectedCar.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>

class FakeCarDetector final : public HubHelper<caf::event_based_actor, void, car_detect_available_atom> {
    Identifier mKey;

public:
    FakeCarDetector(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](image_frame_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame>);
                     DetectedCarArray res;
                     res.frame = BlackBoard::instance().get<CameraFrame>(key).value();

                     res.cars.push_back(cv::Rect{ 0, 0, res.frame.frame.cols, res.frame.frame.rows });

                     sendAll(car_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 } };
    }
};

HUB_REGISTER_CLASS(FakeCarDetector);
