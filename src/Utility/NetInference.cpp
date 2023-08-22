#ifdef ARTINX_OPENVINO2022
#include "NetInference.hpp"

YoloNet::YoloNet(std::string& modelPath, float nmsThreshold, float confThreshold, int imgSize, int kptNum, int classNum,
                 int anchorNum) {
    this->nmsThreshold = nmsThreshold;
    this->confThreshold = confThreshold;
    this->imgSize = imgSize;
    this->kptNum = kptNum;
    this->classNum = classNum;
    this->anchorNum = anchorNum;
    compiledModel = core.compile_model(modelPath, "AUTO");
    std::map<std::string, std::string> config = { { InferenceEngine::PluginConfigParams::KEY_PERF_COUNT,
                                                    InferenceEngine::PluginConfigParams::YES } };
    inferRequest = compiledModel.create_infer_request();
    inputTensor1 = inferRequest.get_input_tensor(0);
}

int argmax(const float* ptr, int len, int stride) {
    int maxArg = 0;
    int i, j = 0;
    for(i = 0; i < len; i++) {
        if(ptr[j] > ptr[maxArg])
            maxArg = j;
        j += stride;
    }
    return maxArg / stride;
}

cv::Mat YoloNet::letterBox(const cv::Mat& src, int h, int w, std::vector<float>& padd) {
    int inputWidth = src.cols;
    int inputHeight = src.rows;
    int tarWidth = w;
    int tarHeight = h;
    float r = std::min(float(tarHeight) / inputHeight, float(tarWidth) / inputWidth);
    int insideWidth = round(inputWidth * r);
    int insideHeight = round(inputHeight * r);
    int paddWidth = tarWidth - insideWidth;
    int paddHeight = tarHeight - insideHeight;
    cv::Mat resizeImg;
    resize(src, resizeImg, cv::Size(insideWidth, insideHeight));
    paddWidth = paddWidth / 2;
    paddHeight = paddHeight / 2;
    padd.push_back(paddWidth);
    padd.push_back(paddHeight);
    padd.push_back(r);
    int top = int(round(paddHeight - 0.1));
    int bottom = int(round(paddHeight + 0.1));
    int left = int(round(paddWidth - 0.1));
    int right = int(round(paddWidth + 0.1));
    copyMakeBorder(resizeImg, resizeImg, top, bottom, left, right, 0, cv::Scalar(114, 114, 114));
    return resizeImg;
}

cv::Rect YoloNet::scaleBox(cv::Rect box, std::vector<float>& padd, float rawW, float rawH) {
    cv::Rect scaledBox;
    scaledBox.width = box.width / padd[2];
    scaledBox.height = box.height / padd[2];
    scaledBox.x = std::max(std::min((float)((box.x - padd[0]) / padd[2]), (float)(rawW - 1)), 0.f);
    scaledBox.y = std::max(std::min((float)((box.y - padd[1]) / padd[2]), (float)(rawH - 1)), 0.f);
    return scaledBox;
}

std::vector<cv::Point2f> YoloNet::scaleBoxKpt(std::vector<cv::Point2f> points, std::vector<float>& padd, float rawW, float rawH,
                                              int idx) {
    std::vector<cv::Point2f> scaledPoints;
    for(int ii = 0; ii < kptNum; ii++) {
        points[idx * kptNum + ii].x =
            std::max(std::min((points[idx * kptNum + ii].x - padd[0]) / padd[2], (float)(rawW - 1)), 0.f);
        points[idx * kptNum + ii].y =
            std::max(std::min((points[idx * kptNum + ii].y - padd[1]) / padd[2], (float)(rawH - 1)), 0.f);
        scaledPoints.push_back(points[idx * kptNum + ii]);
    }
    return scaledPoints;
}

std::vector<YoloNet::Object> YoloNet::work(cv::Mat srcImg) {
    int imgH = imgSize;
    int imgW = imgSize;
    cv::Mat img;
    std::vector<float> padd;
    cv::Mat boxed = letterBox(srcImg, imgH, imgW, padd);
    cv::cvtColor(boxed, img, cv::COLOR_BGR2RGB);
    auto data = inputTensor1.data<float>();
    for(int h = 0; h < imgH; h++) {
        for(int w = 0; w < imgW; w++) {
            for(int c = 0; c < 3; c++) {
                int outIndex = c * imgH * imgW + h * imgW + w;
                data[outIndex] = float(img.at<cv::Vec3b>(h, w)[c]) / 255.0f;
            }
        }
    }
    inferRequest.start_async();
    inferRequest.wait();
    auto outputTensor = inferRequest.get_output_tensor(0);
    const float* pred = outputTensor.data<const float>();

    int stride[3] = { 8, 16, 32 };
    int grid = 0;
    for(int i : stride) {
        grid += (imgW / i) * (imgH / i);
    }
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<cv::Point2f> points;
    int anchorsNum = anchorNum * grid;
    for(int i = 0; i < anchorsNum; ++i) {
        float x = pred[0 * anchorsNum + i];
        float y = pred[1 * anchorsNum + i];
        float w = pred[2 * anchorsNum + i];
        float h = pred[3 * anchorsNum + i];
        int boxClass;
        float prob;
        if(classNum != 1) {
            boxClass = argmax(pred + 4 * anchorsNum + i, classNum, anchorsNum);
            prob = pred[(4 + boxClass) * anchorsNum + i];
        } else {
            prob = pred[4 * anchorsNum + i];
        }
        if(prob > confThreshold) {
            cv::Rect_<float> rect = { x - w / 2, y - h / 2, w, h };
            classIds.push_back(boxClass);
            confidences.push_back(prob);
            boxes.push_back(rect);
            // kpt behind the x,y,w,h,p in the labelForm
            for(int j = 1; j <= kptNum; j++) {
                cv::Point2f kPoint;
                kPoint.x = pred[(3 + classNum + j) * anchorsNum + i];
                kPoint.y = pred[(3 + classNum + j * 2) * anchorsNum + i];
                points.push_back(kPoint);
            }
        }
    }

    std::vector<int> picked;
    std::vector<Object> objectResult;

    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, picked);
    for(int i : picked) {
        cv::Rect scaledBox = scaleBox(boxes[i], padd, srcImg.cols, srcImg.rows);
        std::vector<cv::Point2f> scaledPoint;
        if(kptNum != 0)
            scaledPoint = scaleBoxKpt(points, padd, srcImg.cols, srcImg.rows, i);
        Object obj;
        obj.rect = scaledBox;
        obj.label = classIds[i];
        obj.prob = confidences[i];
        if(kptNum != 0)
            obj.kpt = scaledPoint;
        objectResult.push_back(obj);
    }
    return objectResult;
}
#endif