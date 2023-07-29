#ifndef ARTINX_RADAR
#include "BlackBoard.hpp"
#include "ClassifiedNum.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "NetInference.hpp"
#include "SelectedTarget.hpp"
#include "Utility.hpp"

#include <string>
#include <utility>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"

#include <Eigen/Core>

struct NNetArmorDetectorSettings final {
    bool debugView;
    std::string modelPath;
    float nmsThreshold;   // NMS参数
    float confThreshold;  // 置信度参数
    int imgSize;          // 推理图像大小
    int kptNum;
    int classNum;
    int anchorNum;
};

template <class Inspector>
bool inspect(Inspector& f, NNetArmorDetectorSettings& x) {
    return f.object(x).fields(
        f.field("debugView", x.debugView).fallback(false), f.field("nmsThreshold", x.nmsThreshold).fallback(0.4),
        f.field("confThreshold", x.confThreshold).fallback(0.6), f.field("imgSize", x.imgSize).fallback(640),
        f.field("kptNum", x.kptNum).fallback(0), f.field("classNum", x.classNum).fallback(14),
        f.field("modelPath", x.modelPath).fallback("data/weights/armor_detect/best.onnx"),
        f.field("anchorNum", x.anchorNum).fallback(3));
}
// 0:hero 1:engineer 2,3,4:infantry 5:outpost 6:sentry
class NNetArmorDetector final
    : public HubHelper<caf::event_based_actor, NNetArmorDetectorSettings, armor_detect_available_atom, image_frame_atom> {
    Identifier mKey;

    std::unique_ptr<YoloNet> mInfer;
    std::unique_ptr<NumberClassifier> mNumClassifierPtr;

    void debugView(const std::string_view& name, const cv::Mat& src, const std::function<void(cv::Mat&)>& func) {
#ifndef ARTINXHUB_DEBUG
        // return;
#endif

        const auto hash = std::hash<std::string_view>{}(name);
        const Identifier newKey{ mKey.val ^ hash };

        cv::Mat res;
        src.copyTo(res);
        if(res.depth() == CV_8U)
            cv::putText(res, name.data(), { 0, 20 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{ 255 });
        else
            cv::putText(res, name.data(), { 0, 20 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{ 0, 255, 0 });

        func(res);

        CameraFrame frame;
        frame.frame = std::move(res);

        sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(newKey, std::move(frame), name));
    }

public:
    NNetArmorDetector(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        mInfer = std::make_unique<YoloNet>(mConfig.modelPath, mConfig.nmsThreshold, mConfig.confThreshold, mConfig.imgSize,
                                           mConfig.kptNum, mConfig.classNum, mConfig.anchorNum);
    }

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](image_frame_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame, std::string_view>);
                     ACTOR_EXCEPTION_PROBE();

                     const auto t1 = Clock::now();
                     const auto frame = std::get<0>(BlackBoard::instance().get<CameraFrame, std::string_view>(key).value());
                     DetectedArmorArray res;
                     res.frame = frame;

                     auto result = mInfer->work(res.frame.frame);
                     logInfo(fmt::format("NNetArmorDetect result size: {}", result.size()));
                     if(mConfig.debugView) {
                         debugView("result", res.frame.frame, [&](cv::Mat& src) {
                             for(size_t i = 0; i < result.size(); ++i) {
                                 cv::rectangle(src, result[i].rect, cv::Scalar(0, 255, 0));
                             }
                         });
                     }

                     if(res.armors.size() > 0) {
                         HubLogger::visualLog(fmt::format("ArmorDetector detected {} targets, cost time {:.3f}ms",
                                                          res.armors.size(), durationCastDouble(Clock::now() - t1) * 1000));
                     }
                     sendAll(armor_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 } };
    }
};

HUB_REGISTER_CLASS(NNetArmorDetector);
#endif