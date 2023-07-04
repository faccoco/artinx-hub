#ifdef ARTINX_RADAR
#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "RadarInfo.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <infer.hpp>
#include <yolo.hpp>

#include "SuppressWarningEnd.hpp"

#include <caf/actor_config.hpp>
#include <exception>
#include <opencv2/core.hpp>
#include <optional>
#include <string>

struct RadarNNetDetectorSettings final {
    std::string modelPath;
    double minConfidence = 0.7;
};

template <typename Inspector>
bool inspect(Inspector& f, RadarNNetDetectorSettings& x) {
    return f.object(x).fields(f.field("modelPath", x.modelPath), f.field("minConfidence", x.minConfidence));
}

class RadarNNetDetector final
    : public HubHelper<caf::event_based_actor, RadarNNetDetectorSettings, bots_locate_request_atom, image_frame_atom> {
    Identifier mKey;
    std::shared_ptr<yolo::Infer> model;

    void debugView(const std::string_view& name, const cv::Mat& src, const std::function<void(cv::Mat&)>& func) {

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
    RadarNNetDetector(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        if(!fs::exists(mConfig.modelPath)) {
            const auto err = "Model not exist at: " + mConfig.modelPath;
            logError(err);
            throw std::runtime_error(err.c_str());
        }
        try {
            model = yolo::load(mConfig.modelPath, yolo::Type::V7);
        } catch(std::exception& e) {
            logError(e.what());
        }
    }

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [this](image_frame_atom, Identifier key) {
                     if(auto data = BlackBoard::instance().get<CameraFrame, std::string_view>(key)) {
                         const auto& [frame, name] = data.value();
                         auto tmp = yolo::Image(frame.frame.data, frame.frame.cols, frame.frame.rows);
                         auto inferRes = model->forward(tmp);
                         cv::Mat debug;
                         frame.frame.copyTo(debug);
                         DetectedBots result;
                         result.frame = frame;
                         std::vector<std::string> pointsName;
                         for(auto& each : inferRes) {
                             if(each.confidence < mConfig.minConfidence)
                                 continue;
                             if(each.class_label != 0)
                                 continue;
                             cv::rectangle(debug, cv::Point2f(each.left, each.top), cv::Point2f(each.right, each.bottom),
                                           cv::Scalar(0, 255, 0), 1);
                             result.botBoxes.push_back(each);
                         }

                         debugView("Detected", debug, [](auto&) {});
                         std::sort(result.botBoxes.begin(), result.botBoxes.end(),
                                   [](const auto& lhs, const auto& rhs) { return lhs.left < rhs.left; });
                         if(result.botBoxes.empty())
                             return;
                         ACTOR_PROTOCOL_CHECK(bots_locate_request_atom, TypedIdentifier<DetectedBots>);
                         sendAll(bots_locate_request_atom_v, BlackBoard::instance().updateSync(mKey, std::move(result)));
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(RadarNNetDetector);
#endif
