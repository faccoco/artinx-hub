#ifdef ARTINX_OPENVINO2022
#include <vector>
#include <string>
#include <cmath>
#include <cstdlib>
#include <cassert>
#include <cfloat>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <sys/types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <openvino/openvino.hpp>
#include <inference_engine.hpp>
using namespace InferenceEngine;

class YoloNet {
public:
    YoloNet() = default;
    explicit YoloNet(std::string& modelPath, float nmsThreshold, float confThreshold, int imgSize,
                      int kptNum, int classNum,  int anchorNum);
    float nmsThreshold; //NMS参数
    float confThreshold;//置信度参数
    int imgSize;  //推理图像大小，如果不是640 和 416 需要自己在下面添加anchor
    int kptNum;
    int classNum;
    int anchorNum;
    std::string modelPath;
    struct Object {
        cv::Rect_<float> rect;
        float x, y, w, h;
        int label;
        float prob;
        std::vector<cv::Point2f>kpt;
    };
    cv::Mat letterBox(const cv::Mat &src, int h, int w, std::vector<float> &padd);

    std::vector<cv::Point2f> scaleBoxKpt(std::vector<cv::Point2f> points, std::vector<float> &padd, float rawW, float rawH, int idx);

    cv::Rect scaleBox(cv::Rect box, std::vector<float> &padd, float rawW, float rawH);

    std::vector<Object> work(cv::Mat srcImg);

private:
    ov::Core core;
    std::shared_ptr<ov::Model> model;
    ov::CompiledModel compiledModel;
    ov::InferRequest inferRequest;
    ov::Tensor inputTensor1;
};
#endif
