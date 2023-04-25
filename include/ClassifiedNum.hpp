#pragma once

#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "DetectedArmor.hpp"

// 0-8 : Base 1 2 3 4 5 sentry Outpost  Negative
class NumberClassifier {
public:
    explicit NumberClassifier(const std::string& modelPath) ;

    static cv::Mat extractNumbers(const cv::Mat& src, const cv::Point2f points[], bool isLargeArmor);

    std::pair<int, float> classify(const cv::Mat& img);

private:
    cv::dnn::Net net;

};
