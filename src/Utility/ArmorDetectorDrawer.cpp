#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include "Utility.hpp"
#include "magic_enum.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

#include "SuppressWarningEnd.hpp"

class ArmorDetectorDrawer final : public HubHelper<caf::event_based_actor, void, image_frame_atom> {
    Identifier mKey;
    cv::Mat mImage;

public:
    ArmorDetectorDrawer(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, std::move(name) }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](armor_detect_available_atom, Identifier key) {
                     auto res = BlackBoard::instance().get<DetectedArmorArray>(key).value();

                     cv::Mat showImg;
                     res.frame.frame.copyTo(showImg);
                     res.frame.frame.copyTo(mImage);

                     int x = static_cast<int>(showImg.size().width),
                         y = static_cast<int>(showImg.size().height);  // 绘制十字瞄准线
                     cv::line(showImg, cv::Point2i(x / 2, 0), cv::Point2i(x / 2, y), { 0, 255, 0 }, 1);
                     cv::line(showImg, cv::Point2i(0, y / 2), cv::Point2i(x, y / 2), { 0, 255, 0 }, 1);
                     for(const auto& armor : res.armors) {
                         // 绘制四点
                         for(int i = 0; i < 4; i++) {
                             cv::putText(showImg, std::to_string(i), armor.light4Point[i], cv::FONT_HERSHEY_SIMPLEX, 1.0,
                                         cv::Scalar(255, 255, 255), 1);
                         }

                         // 绘制装甲板四点矩形
                         for(int i = 0; i < 4; i++) {
                             cv::line(showImg, armor.light4Point[i], armor.light4Point[(i + 1) % 4], cv::Scalar(0, 255, 255), 1);
                         }

                         // 绘制目标颜色与类别
                         std::string id(fmt::format("{} {}", magic_enum::enum_name(armor.robotType), armor.isLargeArmor));
                         int box_top_x = static_cast<int>(armor.light4Point[0].x);
                         int box_top_y = static_cast<int>(armor.light4Point[0].y);

                         cv::putText(showImg, id, cv::Point(box_top_x + 2, box_top_y), cv::FONT_HERSHEY_TRIPLEX, 0.5,
                                     cv::Scalar(0, 255, 255));
                     }

                     CameraFrame frame;
                     frame.frame = std::move(showImg);
                     frame.lastUpdate = res.frame.lastUpdate;
                     frame.info = res.frame.info;

                     sendAll(image_frame_atom_v,
                             BlackBoard::instance().updateSync(mKey, std::move(frame), std::string_view("ArmorDetectorDrawer")));
                 },
                 [&](set_projected_target_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(set_projected_target_atom, TypedIdentifier<ProjectedTarget>);
                     ACTOR_EXCEPTION_PROBE();
                     auto res = BlackBoard::instance().get<ProjectedTarget>(key).value();
                     auto projectedPoints = res.projectedPoints;

                     auto name = "ProjectionDrawer";
                     const auto hash = std::hash<std::string_view>{}(name);
                     const Identifier newKey{ mKey.val ^ hash };

                     if(mImage.empty()) {
                         return;
                     }

                     cv::Mat showImg;
                     mImage.copyTo(showImg);

                     for (int i = 0; i < res.armorCorners.size(); i++) {
                         cv::line(showImg, res.armorCorners[i], res.armorCorners[(i + 1) % 4], cv::Scalar(0, 255, 255), 1);
                     }

                     for (const auto& point : projectedPoints.first) {
                         cv::circle(showImg, point, 2, cv::Scalar(0, 255, 0), 2);
                     }

                    for (const auto& point : projectedPoints.second) {
                        cv::circle(showImg, point, 2, cv::Scalar(0, 255, 255), 2);
                    }


                     CameraFrame frame;
                     frame.frame = std::move(showImg);
                     frame.lastUpdate = res.lastUpdate;

                     sendAll(image_frame_atom_v,
                             BlackBoard::instance().updateSync(newKey, std::move(frame), std::string_view(name)));
                 } };
    }
};

HUB_REGISTER_CLASS(ArmorDetectorDrawer);
