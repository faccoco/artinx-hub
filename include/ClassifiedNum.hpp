#pragma once

#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "DetectedArmor.hpp"

class NumberClassifier {
public:
    NumberClassifier(const std::string& modelPath, const std::string& labelPath, double threshold);

    std::vector<cv::Mat> extractNumbers(const cv::Mat& src, const std::vector<Armor>& armors);

    void classify(std::vector<Armor>& armors, const std::vector<cv::Mat>& imgs);

    double threshold;

private:
    cv::dnn::Net net;
    std::vector<std::string> className;
};
