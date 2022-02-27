#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <fmt/format.h>
#include <opencv2/mcc.hpp>

struct ColorCalibratorSettings final {
    double maxFittingLoss;
    bool ACESToneMapping;
    double toneMappingLumFactor;
};

template <class Inspector>
bool inspect(Inspector& f, ColorCalibratorSettings& x) {
    return f.object(x).fields(
        f.field("maxFittingLoss", x.maxFittingLoss).invariant([](double v) { return v > 5.0 && v < 15.0; }),
        f.field("ACESToneMapping", x.ACESToneMapping),
        f.field("toneMappingLumFactor", x.toneMappingLumFactor).invariant([](double v) { return v > 0.0 && v < 3.0; }));
}

class ColorCalibrator final : public HubHelper<caf::event_based_actor, ColorCalibratorSettings, image_frame_atom> {
    cv::Ptr<cv::mcc::CCheckerDetector> mDetector = cv::mcc::CCheckerDetector::create();

    using CalibrationData = cv::ccm::ColorCorrectionModel;

    Identifier mKey;
    mutable std::optional<CalibrationData> mCalibratedData;

    void detectColorCheckerAndCalibrate(const cv::Mat& frame) {
        const auto& data = mCalibratedData.value();
        if(!mDetector->process(frame, cv::mcc::MCC24, 1, true))
            return;

        cv::Ptr<cv::mcc::CChecker> checker = mDetector->getBestColorChecker();
        const auto chartsRGB = checker->getChartsRGB();
        const auto src = chartsRGB.col(1).clone().reshape(3, chartsRGB.rows / 3);
        src /= 255.0;
        cv::ccm::ColorCorrectionModel model{ src, cv::ccm::COLORCHECKER_Macbeth };
        model.run();
        const auto loss = model.getLoss();
        if(loss >= mConfig.maxFittingLoss)
            return;

        mCalibratedData = std::move(model);
    }

    cv::Mat applyACES(const cv::Mat& frame) const {
        // Please refer to https://knarkowicz.wordpress.com/2016/01/06/aces-filmic-tone-mapping-curve/
        constexpr double a = 2.51, b = 0.03, c = 2.43, d = 0.59f, e = 0.14;
        const auto meanColor = cv::mean(frame);
        const auto lum = meanColor[0] * 0.2126 + meanColor[1] * 0.7152 + meanColor[2] * 0.0722;
        const auto x = frame * (lum * mConfig.toneMappingLumFactor);
        return (x * (a * x + b)) / (x * (c * x + d) + e);
    }

    cv::Mat applyCalibration(const cv::Mat& frame) const {
        // TODO: reduce reallocation
        auto& model = mCalibratedData.value();
        cv::Mat image;
        cv::cvtColor(frame, image, cv::COLOR_BGR2RGB);
        image.convertTo(image, CV_64F);
        image /= 255;
        const auto calibratedImage = model.infer(image);
        const auto scaled = mConfig.ACESToneMapping ? applyACES(calibratedImage) : calibratedImage;

        cv::Mat outFloat = cv::min(cv::max(calibratedImage * 255, 0), 255);
        cv::Mat out24Bit;
        outFloat.convertTo(out24Bit, CV_8UC3);
        cv::Mat outImage;
        cv::cvtColor(out24Bit, outImage, cv::COLOR_RGB2BGR);
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
                     }

                     BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(image_frame_atom_v, mKey);
                 } };
    }
};

HUB_REGISTER_CLASS(ColorCalibrator);
