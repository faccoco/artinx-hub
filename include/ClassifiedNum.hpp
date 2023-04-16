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

    void extractNumbers(const cv::Mat& src, std::vector<Armor>& armors);

    void classify(std::vector<Armor>& armors);

    double threshold;

private:
    cv::dnn::Net net;
    std::vector<std::string> className;
};
