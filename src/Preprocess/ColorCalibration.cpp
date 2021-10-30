#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <fmt/format.h>
#include <opencv2/mcc.hpp>

struct ColorCalibratorSettings final {};

template <class Inspector>
bool inspect(Inspector& f, ColorCalibratorSettings& x) {
    return f.object(x).fields();
}

class ColorCalibrator final : public HubHelper<caf::event_based_actor, ColorCalibratorSettings, image_frame_atom> {
    struct CalibrationData final {
        mutable cv::ccm::ColorCorrectionModel model;
        cv::Ptr<cv::mcc::CCheckerDetector> detector = cv::mcc::CCheckerDetector::create();
    };

    Identifier mKey;
    std::optional<CalibrationData> mCalibratedData;

    void detectColorCheckerAndCalibrate(const cv::Mat& frame) {
        const auto& data = mCalibratedData.value();
        if(!data.detector->process(frame, cv::mcc::MCC24, 1, true)) {
            CAF_LOG_INFO("ChartColor not detected \n");
            return;
        }
        cv::Ptr<cv::mcc::CChecker> checker = data.detector->getBestColorChecker();
        const auto chartsRGB = checker->getChartsRGB();
        const auto src = chartsRGB.col(1).clone().reshape(3, chartsRGB.rows / 3);
        src /= 255.0;
        cv::ccm::ColorCorrectionModel model(src, cv::ccm::COLORCHECKER_Macbeth);
        model.run();
        const auto loss = model.getLoss();
        if(loss >= 8) {
            CAF_LOG_INFO(fmt::format("loss:{:.2f}", loss));
            return;
        }
        mCalibratedData->model = model;
    }

    cv::Mat applyCalibration(const cv::Mat& frame) const {
        const auto& data = mCalibratedData.value();
        cv::Mat image;
        cv::cvtColor(frame, image, cv::COLOR_BGR2RGB);
        image.convertTo(image, CV_64F);
        image /= 255;
        const auto calibratedImage = data.model.infer(image);
        cv::Mat out = calibratedImage * 255;
        out.convertTo(out, CV_8UC3);
        cv::Mat outImage;
        cv::cvtColor(cv::min(cv::max(out, 0), 255), outImage, cv::COLOR_RGB2BGR);
        return outImage;
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
