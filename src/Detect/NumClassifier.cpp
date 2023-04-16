#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "ClassifiedNum.hpp"

NumberClassifier::NumberClassifier(const std::string& modelPath, const std::string& labelPath, double thre) : threshold(thre) {
    net = cv::dnn::readNetFromONNX(modelPath);

    std::ifstream labelFile(labelPath);
    std::string line;
    while(std::getline(labelFile, line)) {
        className.push_back(line);
    }
}

std::vector<cv::Mat> NumberClassifier::extractNumbers(const cv::Mat& src, const std::vector<Armor>& armors) {
    // Light length in image
    constexpr int lightLen = 12;
    // Image size after warp
    constexpr int warpHeight = 28;
    constexpr int smallArmorWidth = 32;
    constexpr int largeArmorWidth = 54;
    // Number ROI size
    const cv::Size roiSize(20, 28);

    std::vector<cv::Mat> numImgs;
    for(auto& armor : armors) {
        // Warp perspective transform
        cv::Point2f lightsVertices[4] = { armor.leftLight.bottom, armor.leftLight.top, armor.rightLight.top,
                                          armor.rightLight.bottom };
        const int topLightY = (warpHeight - lightLen) / 2 - 1;
        const int bottomLightY = topLightY + lightLen;
        const int warpWidth = armor.armorType == ArmorType::Small ? smallArmorWidth : largeArmorWidth;
        cv::Point2f target_vertices[4] = {
            cv::Point(0, bottomLightY),
            cv::Point(0, topLightY),
            cv::Point(warpWidth - 1, topLightY),
            cv::Point(warpWidth - 1, bottomLightY),
        };
        cv::Mat numberImg;
        auto rotation_matrix = cv::getPerspectiveTransform(lightsVertices, target_vertices);
        cv::warpPerspective(src, numberImg, rotation_matrix, cv::Size(warpWidth, warpHeight));

        // Get ROI
        numberImg = numberImg(cv::Rect(cv::Point((warpWidth - roiSize.width) / 2, 0), roiSize));

        // Binarize
        cv::cvtColor(numberImg, numberImg, cv::COLOR_RGB2GRAY);
        cv::threshold(numberImg, numberImg, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        numImgs.emplace_back(std::move(numberImg));
    }
    return numImgs;
}

void NumberClassifier::classify(std::vector<Armor>& armors, const std::vector<cv::Mat>& imgs) {
    for(u_int32_t i = 0; i < armors.size(); ++i) {
        cv::Mat image = imgs[i].clone();

        // Normalize
        image = image / 255.0;

        // Create blob from image
        cv::Mat blob;
        cv::dnn::blobFromImage(image, blob, 1., cv::Size(28, 20));

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

        armors[i].id = labelId;
        armors[i].confidence = confidence;
        armors[i].armorName = className[labelId];
    }

    armors.erase(std::remove_if(armors.begin(), armors.end(),
                                [this](const Armor& armor) {
                                    if(armor.confidence < threshold || armor.armorName == "N") {
                                        return true;
                                    }

                                    bool mismatchArmorType = false;
                                    if(armor.armorType == ArmorType::Large) {
                                        mismatchArmorType =
                                            armor.armorName == "Outpost" || armor.armorName == "2" || armor.armorName == "Guard";
                                    } else if(armor.armorType == ArmorType::Large) {
                                        mismatchArmorType = armor.armorName == "1" || armor.armorName == "Base";
                                    }
                                    return mismatchArmorType;
                                }),
                 armors.end());
}
