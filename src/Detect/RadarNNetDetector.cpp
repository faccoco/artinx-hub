#ifdef ARTINX_RADAR
#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "RadarInfo.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <infer.hpp>
#include <yolo.hpp>

#include "SuppressWarningEnd.hpp"

#include <caf/actor_config.hpp>
#include <string_view>

struct RadarNNetDetectorSettings final {
    std::string modelPath;
};

template <typename Inspector>
bool inspect(Inspector& f, RadarNNetDetectorSettings& x) {
    return f.object(x).fields(f.field("modelPath", x.modelPath));
}

class RadarNNetDetector final
    : public HubHelper<caf::event_based_actor, RadarNNetDetectorSettings, bots_locate_succeed_atom, image_frame_atom> {
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
            const auto err = "Model not exist: " + mConfig.modelPath;
            logError(err);
            throw std::runtime_error(err.c_str());
        }
        model = yolo::load(mConfig.modelPath, yolo::Type::V7);
    }

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [this](image_frame_atom, Identifier key) {
                     if(auto data = BlackBoard::instance().get<CameraFrame, std::string_view>(key)) {
                         auto start = std::chrono::system_clock::now();
                         const auto& [frame, name] = data.value();
                         auto tmp = yolo::Image(frame.frame.data, frame.frame.cols, frame.frame.rows);
                         auto res = model->forward(tmp);
                         logWarning(std::to_string(res.size()));
                         cv::Mat debug(frame.frame);
                         for(auto& each : res) {
                             cv::rectangle(debug, cv::Point2f(each.left, each.top), cv::Point2f(each.right, each.bottom),
                                           cv::Scalar(0, 0, 255), 2);
                         }
                         debugView("detected", debug, [](auto&) {});
                         auto end = std::chrono::system_clock::now();
                         std::chrono::duration<double> elapsed_seconds = end - start;
                         logInfo("infer rate" + std::to_string(1 / elapsed_seconds.count()) + " HZ");
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(RadarNNetDetector);
#endif
