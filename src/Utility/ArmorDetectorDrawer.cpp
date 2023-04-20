#include "BlackBoard.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "Hub.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>

#include "SuppressWarningEnd.hpp"
#include <Eigen/Core>

class ArmorDetectorDrawer final : public HubHelper<caf::event_based_actor, void, image_frame_atom> {
    Identifier mKey;

public:
    ArmorDetectorDrawer(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](armor_detect_available_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(armor_detect_available_atom, TypedIdentifier<DetectedArmorArray>);
                     const auto data = BlackBoard::instance().get<DetectedArmorArray>(key).value();
                     cv::Mat labeled;
                     data.frame.frame.copyTo(labeled);

                     const cv::Scalar red{ 0, 0, 255 };
                     const cv::Scalar green{ 0, 255, 0 };

                     for (auto&& armor : data.armors){
                         std::vector<cv::Point2f> imgPoints{ armor.leftLight.top, armor.leftLight.bottom, armor.rightLight.bottom,
                                                             armor.rightLight.top };
                         for (int i = 0; i < 4; ++i){
                            cv::circle(labeled, imgPoints[i], 2, green);
                            cv::line(labeled, imgPoints[i], imgPoints[(i + 1) % 4], red);
                         }

                         cv::putText(labeled, fmt::format("{} : {:.2f}", armor.armorName.c_str(), armor.confidence), { static_cast<int>(armor.center.x), static_cast<int>(armor.leftLight.top.y) }, cv::FONT_HERSHEY_SIMPLEX,
                                     0.5, green);
                     }

                     CameraFrame frame;
                     frame.frame = std::move(labeled);
                     frame.lastUpdate = data.frame.lastUpdate;
                     frame.info = data.frame.info;

                     sendAll(image_frame_atom_v,
                             BlackBoard::instance().updateSync(mKey, std::move(frame), std::string_view("ArmorDetectorDrawer")));
                 },
                 [&](armor_nnet_detect_available_atom, Identifier key) {
                     auto res = BlackBoard::instance().get<NNetDetectedArmorArray>(key).value();

                     cv::Mat showImg;
                     res.frame.frame.copyTo(showImg);
                     for(const auto& armor : res.armors) {
                         // 绘制十字瞄准线
                         cv::line(showImg, cv::Point2f(showImg.size().width / 2, 0),
                                  cv::Point2f(showImg.size().width / 2, showImg.size().height), { 0, 255, 0 }, 1);
                         line(showImg, cv::Point2f(0, showImg.size().height / 2),
                              cv::Point2f(showImg.size().width, showImg.size().height / 2), { 0, 255, 0 }, 1);

                         // 绘制四点
                         for(int i = 0; i < 4; i++) {
                             cv::circle(showImg, cv::Point(armor.light4Point[i].x, armor.light4Point[i].y), 1,
                                        cv::Scalar(100, 200, 0), 1);
                         }

                         // 绘制装甲板四点矩形
                         for(int i = 0; i < 4; i++) {
                             cv::line(showImg, armor.light4Point[i], armor.light4Point[(i + 1) % 4], cv::Scalar(100, 200, 0), 1);
                         }

                         // 绘制目标颜色与类别
                         int id = armor.robotType;
                         int box_top_x = armor.light4Point[0].x;
                         int box_top_y = armor.light4Point[0].y;
                         if(armor.robotColor == 0)
                             cv::putText(showImg, "Blue_" + std::to_string(id), cv::Point(box_top_x + 2, box_top_y),
                                         cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255, 0, 0));
                         else if(armor.robotColor == 1)
                             cv::putText(showImg, "Red_" + std::to_string(id), cv::Point(box_top_x + 2, box_top_y),
                                         cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(0, 0, 255));
                         else if(armor.robotColor == 2)
                             cv::putText(showImg, "None_" + std::to_string(id), cv::Point(box_top_x + 2, box_top_y),
                                         cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(0, 255, 0));
                     }

                     CameraFrame frame;
                     frame.frame = std::move(showImg);
                     frame.lastUpdate = res.frame.lastUpdate;
                     frame.info = res.frame.info;

                     sendAll(image_frame_atom_v,
                             BlackBoard::instance().updateSync(mKey, std::move(frame), std::string_view("ArmorDetectorDrawer")));
                 } };
    }
};

HUB_REGISTER_CLASS(ArmorDetectorDrawer);
