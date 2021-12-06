#include "BlackBoard.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <caf/event_based_actor.hpp>

class ArmorDetectorDrawer final : public HubHelper<caf::event_based_actor, void, image_frame_atom> {
    Identifier mKey;

public:
    ArmorDetectorDrawer(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(ArmorDetectorDrawer).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](armor_detect_available_atom, Identifier key) {
                     const auto data = BlackBoard::instance().get<DetectedArmorArray>(key).value();
                     cv::Mat labeled;
                     data.frame.frame.copyTo(labeled);

                     const cv::Scalar red{ 0, 0, 255 };
                     const cv::Scalar green{ 255, 0, 0 };

                     for(auto& car : data.armors) {
                         for(auto& armor : car.armors) {
                             auto r1 = armor.r1, r2 = armor.r2;
                             r1.center += cv::Point2f{ car.roi.tl() };
                             r2.center += cv::Point2f{ car.roi.tl() };

                             drawRotatedRect(labeled, r1, green);
                             drawRotatedRect(labeled, r2, green);

                             cv::Point2f pts[4];
                             std::vector<cv::Point2f> pts8;
                             pts8.reserve(8);
                             r1.points(pts);
                             pts8.insert(pts8.cend(), pts, pts + 4);
                             r2.points(pts);
                             pts8.insert(pts8.cend(), pts, pts + 4);

                             drawRotatedRect(labeled, cv::minAreaRect(pts8), red);
                         }
                     }

                     CameraFrame frame;
                     frame.frame = std::move(labeled);
                     frame.lastUpdate = data.frame.lastUpdate;
                     frame.info = data.frame.info;

                     BlackBoard::instance().updateSync(mKey, std::move(frame));
                     sendAll(image_frame_atom_v, mKey);
                 } };
    }
};

HUB_REGISTER_CLASS(ArmorDetectorDrawer);
