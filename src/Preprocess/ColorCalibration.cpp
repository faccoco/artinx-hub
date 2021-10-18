#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <opencv2/mcc.hpp>

struct ColorCalibratorSettings final {};

template <class Inspector>
bool inspect(Inspector& f, ColorCalibratorSettings& x) {
    return f.object(x).fields();
}

class ColorCalibrator final : public HubHelper<caf::event_based_actor, ColorCalibratorSettings, image_frame_atom> {
    struct CalibrationData final {
        // Implement here
    };

    Identifier mKey;
    std::optional<CalibrationData> mCalibratedData;

    void detectColorCheckerAndCalibrate(const cv::Mat& frame) {
        // Implement here
    }

    cv::Mat applyCalibration(const cv::Mat& frame) const {
        const auto& data = mCalibratedData.value();
        // Implement here
        return frame;
    }

public:
    ColorCalibrator(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(ColorCalibrator).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](image_frame_atom, Identifier key) {
                     auto res = BlackBoard::instance().get<CameraFrame>(key).value();

                     if(mCalibratedData.has_value()) {
                         res.frame = applyCalibration(res.frame);
                     } else {
                         detectColorCheckerAndCalibrate(res.frame);
                         if(!mCalibratedData.has_value())
                             CAF_LOG_ERROR("Color checker is not detected!");
                     }

                     BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(image_frame_atom_v, mKey);
                 } };
    }
};

HUB_REGISTER_CLASS(ColorCalibrator);
