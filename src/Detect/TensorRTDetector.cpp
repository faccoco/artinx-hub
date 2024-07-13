#include <chrono>
#include <ratio>
#include <vector>
#ifdef ARTINX_CUDA
#include "BlackBoard.hpp"
#include "ClassifiedNum.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <cstdint>

#include <string>
#include <utility>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"
#include "TRTModule.hpp"

struct TensorRTDetectorSetting final {
    bool debugView;
    std::string onnxPath;
    float numThresh;

    double edgeRatio;
};

constexpr float fontScale = 1.5;

template <class Inspector>
bool inspect(Inspector& f, TensorRTDetectorSetting& x) {
    return f.object(x).fields(f.field("debugView", x.debugView).fallback(false), f.field("onnxPath", x.onnxPath),
                              f.field("numThresh", x.numThresh).fallback(0.7), f.field("edgeRatio", x.edgeRatio).fallback(5.0));
}

class TensorRTDetector final
    : public HubHelper<caf::event_based_actor, TensorRTDetectorSetting, armor_detect_available_atom, image_frame_atom> {
    Identifier mKey;
    std::shared_ptr<TRTModule> mTRTModel;

    void debugView(const std::string_view& name, const cv::Mat& src, const std::function<void(cv::Mat&)>& func) {
#ifndef ARTINXHUB_DEBUG
        // return;
#endif

        const auto hash = std::hash<std::string_view>{}(name);
        const Identifier newKey{ mKey.val ^ hash };

        cv::Mat res;
        src.copyTo(res);
        if(res.depth() == CV_8U) {
            cv::putText(res, name.data(), { 0, 20 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{ 255 });
        } else {
            cv::putText(res, name.data(), { 0, 20 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{ 0, 255, 0 });
        }

        func(res);

        CameraFrame frame;
        frame.frame = std::move(res);

        sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(newKey, std::move(frame), name));
    }

    void copyBBoxToArmor(std::vector<Armor>& armors, std::vector<bbox_t>& detections) {
        auto color = GlobalSettings::get().getColor();
        for(auto box : detections) {
            Armor armor;
            armor.light4Point.insert(armor.light4Point.end(), std::begin(box.pts), std::end(box.pts));
            armor.lightRect = cv::boundingRect(armor.light4Point);
            armor.robotType = static_cast<RobotType>(box.tag_id);
            armor.robotColor = static_cast<Color>(box.color_id);
            armor.prob = box.confidence;
            armor.isLargeArmor = false;

            auto lightWidth = (cv::norm(armor.light4Point[0] - armor.light4Point[1]) + cv::norm(armor.light4Point[2] - armor.light4Point[3])) / 2;
            auto lightLength = (cv::norm(armor.light4Point[0] - armor.light4Point[3]) + cv::norm(armor.light4Point[1] - armor.light4Point[2])) / 2;
            if (lightLength / lightWidth > mConfig.edgeRatio || armor.robotType == RobotType::Hero || armor.robotType == RobotType::Base) {
                armor.isLargeArmor = true;
            }

            if (box.color_id == static_cast<int>(color)){
                armors.push_back(armor); // 1 in box.color_id is blue, however enum class Color is defined enum class Color { Blue, Red, Purple, Negative };
            }
        }
    }

    std::vector<bbox_t> detect(cv::Mat& frame, std::vector<Armor>& armors) {
        std::vector<bbox_t> detections = mTRTModel->operator()(frame);
        if(mConfig.debugView) {
            debugView("conor_points", frame, [&](cv::Mat& src) {
                if(!detections.empty()) {
                    bbox_t box = detections[0];
                    for(int i = 0; i < 4; i++) {
                        cv::circle(src, box.pts[0], 1, cv::Scalar{ 255, 255, 0 });
                        cv::putText(src, std::to_string(i), box.pts[i], cv::FONT_HERSHEY_SIMPLEX, fontScale,
                                    cv::Scalar{ 0, 255, 255 });
                    }
                    cv::line(src, box.pts[0], box.pts[2], cv::Scalar{ 255, 255, 0 }, 2);
                    cv::line(src, box.pts[1], box.pts[3], cv::Scalar{ 255, 255, 0 }, 2);
                }
            });
        }
        copyBBoxToArmor(armors, detections);
        return detections;
    }

//    void postProcess(std::vector<Armor>& armors){
//        for (auto& armor : armors){
//            if ()
//        }
//    }

public:
    TensorRTDetector(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, std::move(name) }, mKey{ generateKey(this) } {
        mTRTModel = std::make_shared<TRTModule>(mConfig.onnxPath);
    }

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](image_frame_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame, std::string_view>);
                     ACTOR_EXCEPTION_PROBE();

                     auto start = std::chrono::high_resolution_clock::now();

                     if(GlobalSettings::get().getTaskMode() != TaskMode::AutoAim) {
                         return;
                     }
                     const auto t1 = Clock::now();
                     const auto data = BlackBoard::instance().get<CameraFrame, std::string_view>(key).value();
                     auto frame = std::get<0>(data);

                     DetectedArmorArray res;
                     res.frame = frame;

                     std::vector<Armor> armors;
                     detect(frame.frame, armors);

                     auto end = std::chrono::high_resolution_clock::now();

                     HubLogger::watch("detect_cost", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

                     res.armors = std::move(armors);

                     if(!armors.empty()) {
                         HubLogger::visualLog(fmt::format("ArmorDetector detected {} targets, cost time {:.3f}ms", armors.size(),
                                                          durationCastDouble(Clock::now() - t1) * 1000));
                     }
                     sendAll(armor_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 } };
    }
};

HUB_REGISTER_CLASS(TensorRTDetector);
#endif
