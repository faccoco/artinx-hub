#include "BlackBoard.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedCar.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <cstdlib>

struct ArmorDetectorTesterSettings final {
    std::string path;
    std::string extension;  // .jpg, .png, etc.
};
template <class Inspector>
bool inspect(Inspector& f, ArmorDetectorTesterSettings& x) {
    return f.object(x).fields(
        f.field("path", x.path).invariant([](const std::string& path) { return fs::exists(path) && fs::is_directory(path); }),
        f.field("extension", x.extension));
}

// NOTICE: ArmorDetector Only
class ArmorDetectorTester final
    : public HubHelper<caf::event_based_actor, ArmorDetectorTesterSettings, car_detect_available_atom, image_frame_atom> {
    Identifier mKey;
    cv::Mat mCurrentFrame;
    uint32_t mCount = 0;

    void next() {
        const auto path = mConfig.path + '/' + std::to_string(mCount) + mConfig.extension;
        if(!fs::exists(path))
            std::exit(EXIT_SUCCESS);

        mCurrentFrame = cv::imread(path);

        DetectedCarArray res;
        res.cars = { cv::Rect{ 0, 0, mCurrentFrame.cols, mCurrentFrame.rows } };
        res.frame.frame = mCurrentFrame;

        BlackBoard::instance().updateSync(mKey, std::move(res));
        sendAll(car_detect_available_atom_v, mKey);

        ++mCount;
    }

public:
    ArmorDetectorTester(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(ArmorDetectorTester).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    Timer::instance().addTimer(caf::actor_cast<caf::actor>(this->address()), 500ms);  // timeout
                    next();
                },
                 [&](armor_detect_available_atom, Identifier key) {
                     const auto data = BlackBoard::instance().get<DetectedArmorArray>(key).value();

                     const cv::Scalar red{ 0, 0, 255 };
                     const cv::Scalar green{ 255, 0, 0 };

                     for(auto& car : data.armors) {
                         for(auto& armor : car.armors) {
                             drawRotatedRect(mCurrentFrame, armor.r1, green);
                             drawRotatedRect(mCurrentFrame, armor.r2, green);

                             cv::Point2f pts[4];
                             std::vector<cv::Point2f> pts8;
                             pts8.reserve(8);
                             armor.r1.points(pts);
                             pts8.insert(pts8.cend(), pts, pts + 4);
                             armor.r2.points(pts);
                             pts8.insert(pts8.cend(), pts, pts + 4);

                             drawRotatedRect(mCurrentFrame, cv::minAreaRect(pts8), red);
                         }
                     }

                     CameraFrame frame;
                     frame.frame = std::move(mCurrentFrame);

                     BlackBoard::instance().updateSync(mKey, std::move(frame));
                     sendAll(image_frame_atom_v, mKey);

                     next();
                 } };
    }
};

HUB_REGISTER_CLASS(ArmorDetectorTester);
