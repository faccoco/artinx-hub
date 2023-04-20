#pragma once

#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "DetectedArmor.hpp"
<<<<<<< HEAD

class NumberClassifier {
public:
    NumberClassifier(const std::string& modelPath, const std::string& labelPath, double threshold);

    void extractNumbers(const cv::Mat& src, std::vector<Armor>& armors);

    void classify(std::vector<Armor>& armors);

    double threshold;

private:
    cv::dnn::Net net;
    std::vector<std::string> className;
=======

class NumberClassifier {
public:
    NumberClassifier(const std::string& modelPath);

    cv::Mat extractNumbers(const cv::Mat& src, const Armor& armor);

    std::pair<int, float> classify(const Armor& armors, const cv::Mat& img);

private:
    cv::dnn::Net net;
>>>>>>> fbe86789e359ff24bb2d5e59f67dab68ceb4611b
};
