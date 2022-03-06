#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "DetectedCar.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>

class FakeCarDetector final : public HubHelper<caf::event_based_actor, void, car_detect_available_atom> {
    Identifier mKey;

public:
    FakeCarDetector(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(FakeCarDetector).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](image_frame_atom, Identifier key) {
                     DetectedCarArray res;
                     res.frame = BlackBoard::instance().get<CameraFrame>(key).value();

                     res.cars.push_back(cv::Rect{ 0, 0, res.frame.frame.cols, res.frame.frame.rows });

                     BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(car_detect_available_atom_v, mKey);
                 } };
    }
};

HUB_REGISTER_CLASS(FakeCarDetector);
