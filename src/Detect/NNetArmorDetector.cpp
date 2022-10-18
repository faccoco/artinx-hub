#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedCar.hpp"
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

#include <inference_engine.hpp>

struct NNetArmorDetectorSettings final {
    std::string networkPath;  // network training file path
};

template <class Inspector>
bool inspect(Inspector& f, NNetArmorDetectorSettings& x) {
    return f.object(x).fields(f.field("networkPath", x.networkPath));
}

class NNetArmorDetector final
    : public HubHelper<caf::event_based_actor, NNetArmorDetectorSettings, armor_detect_available_atom, image_frame_atom> {
    Identifier mKey;
    Core mIe;
    CNNNetork mNetwork;
    Executablenetwork mExeNetwork;
    InferRequest mInferRequest;
    MemoryBlob::CPtr mOutput;
    std::string mInputName;
    std::string mOutputName;
    Eigen::Matrix<float,3,3> mTransfromMatrix;

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

        sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(newKey, std::move(frame)));
    }

    bool initModel(std::string networkPath)
    {

    }
public:
    NNetArmorDetector(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](image_frame_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame>);
                     ACTOR_EXCEPTION_PROBE();

                     const auto frame = BlackBoard::instance().get<CameraFrame>(key).value();

                     NNetDetectedArmorArray res;

                     sendAll(armor_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 } };
    }
};

HUB_REGISTER_CLASS(NNetArmorDetector);
