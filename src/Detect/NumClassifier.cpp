#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "ClassifiedNum.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>
#include <cmath>
#include <cstdint>
#include <fmt/format.h>
#include <inference_engine.hpp>

struct NumClassifierSettings final {
    int32_t inputWidth;
    int32_t inputHeight;
    std::string xmlPath;
    std::string binPath;
    std::string deviceName;
};

namespace IE = InferenceEngine;
constexpr int32_t numCount = 5;

template <class Inspector>
bool inspect(Inspector& f, NumClassifierSettings& x) {
    return f.object(x).fields(f.field("inputWidth", x.inputWidth).fallback(28),
                              f.field("inputHeight", x.inputHeight).fallback(28), f.field("xmlPath", x.xmlPath),
                              f.field("binPath", x.binPath), f.field("deviceName", x.deviceName).fallback("CPU"));
}

class NumClassifier final : public HubHelper<caf::event_based_actor, NumClassifierSettings> {
    Identifier mKey;
    IE::Core mInferenceEngine;
    IE::CNNNetwork mNetwork;
    IE::ExecutableNetwork mExecutableNetwork;
    std::string mInputName, mOutputName;

    void blobFromImage(const cv::Mat& srcImg, IE::Blob::Ptr& inputBlob) {
        cv::Mat dstImg;
        cv::cvtColor(srcImg, dstImg, cv::COLOR_BGR2GRAY);
        cv::resize(srcImg, srcImg, cv::Size(mConfig.inputWidth, mConfig.inputHeight));

        auto inputData = inputBlob->buffer().as<IE::PrecisionTrait<IE::Precision::FP32>::value_type*>();

        for(size_t h = 0; h < mConfig.inputHeight; h++) {
            for(size_t w = 0; w < mConfig.inputWidth; w++) {
                inputData[h * mConfig.inputWidth + w] = (float)dstImg.at<uchar>(h, w) / 255.0f;
            }
        }
    }

    std::tuple<int32_t, double> decodeInferResult(IE::Blob::Ptr& outputBlob) {
        auto outputData = outputBlob->buffer().as<IE::PrecisionTrait<IE::Precision::FP32>::value_type*>();
        double maxTensor = -100, sumExp = 0;
        int32_t resNum = 0;
        for(int i = 0; i < numCount; ++i) {
            if(outputData[i] > maxTensor) {
                resNum = i + 1;
                maxTensor = outputData[i];
            }
            sumExp += std::exp(outputData[i]);
        }
        double confidence = std::exp(outputData[resNum - 1]) / sumExp;
        return std::make_tuple(resNum, confidence);
    }

public:
    NumClassifier(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(NumClassifier).hash_code() } {
        auto [outputBlobName, outputBlob] = *mNetwork.getOutputsInfo().begin();
        mOutputName = outputBlobName;
        outputBlob->setPrecision(IE::Precision::FP32);

        mExecutableNetwork = mInferenceEngine.LoadNetwork(mNetwork, mConfig.deviceName);
        auto [inputName, inputInfo] = *mNetwork.getInputsInfo().begin();
        mInputName = inputName;
        auto [outputName, outputInfo] = *mNetwork.getInputsInfo().begin();
        mOutputName = outputName;
    }

public:
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](num_classify_request_atom, Identifier key) {
                     const auto imgData = BlackBoard::instance().get<CameraFrame>(key).value();
                     auto request = mExecutableNetwork.CreateInferRequest();

                     auto inputBlob = request.GetBlob(mInputName);
                     blobFromImage(imgData.frame, inputBlob);

                     request.Infer();

                     auto outputBlob = request.GetBlob(mOutputName);
                     const auto [num, confidence] = decodeInferResult(outputBlob);
                     return ClassifiedNum{ num, confidence };
                 } };
    }
};

HUB_REGISTER_CLASS(NumClassifier);
