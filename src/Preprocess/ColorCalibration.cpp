#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <opencv2/mcc.hpp>

struct ColorCalibratorSettings final {};

template <class Inspector>
bool inspect(Inspector& f, ColorCalibratorSettings& x) {
    return f.object(x).fields();
}

class ColorCalibrator final : public HubHelper<caf::event_based_actor, ColorCalibratorSettings, image_frame_atom> {
    struct CalibrationData final {
        mutable cv::ccm::ColorCorrectionModel model;
    };

    Identifier mKey;
    std::optional<CalibrationData> mCalibratedData;

    void detectColorCheckerAndCalibrate(const cv::Mat& frame) {
        cv::Mat detectedFrame;
        resize(frame, detectedFrame, cv::Size(800, 600));
        cv::Ptr<cv::mcc::CCheckerDetector> detector = cv::mcc::CCheckerDetector::create();
        detector->process(detectedFrame, cv::mcc::MCC24, 1, false, cv::mcc::DetectorParameters::create());
        cv::Ptr<cv::mcc::CChecker> checker = detector->getBestColorChecker();
        cv::Mat chartsRGB = checker->getChartsRGB();
        cv::Mat src = chartsRGB.col(1).clone().reshape(3, chartsRGB.rows / 3);
        src /= 255.0;
        cv::ccm::ColorCorrectionModel model1(src, cv::ccm::COLORCHECKER_Macbeth);
        model1.run();
        double loss = model1.getLoss();
        if(loss>=8) {
            std::cout << "loss: " << loss << std::endl;
            return ;
        }
        mCalibratedData->model = model1;
    }

    cv::Mat applyCalibration(const cv::Mat& frame) const {
        const auto& data = mCalibratedData.value();
        cv::Mat img_;
        cvtColor(frame, img_, cv::COLOR_BGR2RGB);
        img_.convertTo(img_, CV_64F);
        const int inp_size = 255;
        const int out_size = 255;
        img_ = img_ / inp_size;
        cv::Mat calibratedImage = data.model.infer(img_);
        cv::Mat out_ = calibratedImage * out_size;
        out_.convertTo(out_, CV_8UC3);
        cv::Mat img_out = min(max(out_, 0), out_size);
        cv::Mat out_img;
        cvtColor(img_out, out_img, cv::COLOR_RGB2BGR);
        return out_img;
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
