#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <string>
#include <vector>

#include "ClassifiedNum.hpp"

NumberClassifier::NumberClassifier(const std::string& modelPath) {
    net = cv::dnn::readNetFromONNX(modelPath);
}

cv::Mat NumberClassifier::extractNumbers(const cv::Mat& src, const cv::Point2f points[], bool isLargeArmor) {
    // Light length in image
    constexpr int lightLen = 12;
    // Image size after warp
    constexpr int warpHeight = 28;
    constexpr int smallArmorWidth = 32;
    constexpr int largeArmorWidth = 54;
    // Number ROI size
    const cv::Size roiSize(20, 28);

    // Warp perspective transform
    const int topLightY = (warpHeight - lightLen) / 2 - 1;
    const int bottomLightY = topLightY + lightLen;
    const int warpWidth = isLargeArmor ? largeArmorWidth : smallArmorWidth;
    cv::Point2f targetVertices[4] = {
        cv::Point(0, topLightY),
        cv::Point(0, bottomLightY),
        cv::Point(warpWidth - 1, bottomLightY),
        cv::Point(warpWidth - 1, topLightY),
    };
    cv::Mat numberImg;
    auto rotation_matrix = cv::getPerspectiveTransform(points, targetVertices);
    cv::warpPerspective(src, numberImg, rotation_matrix, cv::Size(warpWidth, warpHeight));
    // Get ROI
    numberImg = numberImg(cv::Rect(cv::Point((warpWidth - roiSize.width) / 2, 0), roiSize));

    // Binarize
    cv::cvtColor(numberImg, numberImg, cv::COLOR_RGB2GRAY);
    cv::threshold(numberImg, numberImg, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    return numberImg;
}

std::pair<int, float> NumberClassifier::classify(const cv::Mat& img) {

    // Normalize
    cv::Mat image = img.clone();
    image /= 255.0;

    // cv::copyMakeBorder(image, image, 0, 0, 4, 4, cv::BORDER_CONSTANT, cv::Scalar(0));

    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob, 1., cv::Size(20, 28));

    // Set the input blob for the neural network
    net.setInput(blob);
    // Forward pass the image blob through the model
    cv::Mat outputs = net.forward();

    // Do softmax
    float maxProb = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::Mat softmaxProb;
    cv::exp(outputs - maxProb, softmaxProb);
    float sum = static_cast<float>(cv::sum(softmaxProb)[0]);
    softmaxProb /= sum;

    double confidence;
    cv::Point classIdPoint;
    minMaxLoc(softmaxProb.reshape(1, 1), nullptr, &confidence, nullptr, &classIdPoint);
    int labelId = classIdPoint.x;

    return { labelId, confidence };
}
