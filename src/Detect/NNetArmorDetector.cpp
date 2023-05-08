#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include "Utility.hpp"

#include <string>
#include <utility>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"

#include <Eigen/Core>
#include <inference_engine.hpp>

namespace IE = InferenceEngine;

struct NNetArmorDetectorSettings final {
    bool debugView;
    std::string networkPath;  // network training file path
    int inputWidth;
    int inputHeight;
    int numClasses;        // Number of classes 8
    int numColors;         // Number of color 4
    float bboxConfThresh;  // 0.6
    uint32_t topK;         // TopK
    float nmsThresh;       // 0.3
    int binaryThresh;
};

template <class Inspector>
bool inspect(Inspector& f, NNetArmorDetectorSettings& x) {
    return f.object(x).fields(
        f.field("debugView", x.debugView).fallback(false), f.field("networkPath", x.networkPath),
        f.field("inputWidth", x.inputWidth), f.field("inputHeight", x.inputHeight), f.field("numClasses", x.numClasses),
        f.field("numColors", x.numColors), f.field("bboxConfThresh", x.bboxConfThresh), f.field("topK", x.topK),
        f.field("nmsThresh", x.nmsThresh).fallback(100), f.field("binaryThresh", x.binaryThresh).fallback(100));
}

struct GridAndStride final {
    int grid0;
    int grid1;
    int stride;
};

// 机器人类别（0：哨兵，1：英雄，2：工程，3、4、5：步兵，6：前哨站，7：基地）
class NNetArmorDetector final
    : public HubHelper<caf::event_based_actor, NNetArmorDetectorSettings, armor_detect_available_atom, image_frame_atom> {
    Identifier mKey;

    IE::Core mIe;
    IE::CNNNetwork mNetwork;
    IE::ExecutableNetwork mExeNetwork;
    IE::InferRequest mInferRequest;

    InferenceEngine::MemoryBlob::Ptr mInputMemBlobPtr;
    InferenceEngine::MemoryBlob::CPtr mOutputMemBlobPtr;

    Eigen::Matrix<float, 3, 3> mTransformMatrix;

    void debugView(const std::string_view& name, const cv::Mat& src, const std::function<void(cv::Mat&)>& func) {
#ifndef ARTINXHUB_DEBUG
        // return;
#endif

        const auto hash = std::hash<std::string_view>{}(name);
        const Identifier newKey{ mKey.val ^ hash };

        cv::Mat res;
        src.copyTo(res);
        if(res.depth() == CV_8U)
            cv::putText(res, name.data(), { 0, 20 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{ 255 });
        else
            cv::putText(res, name.data(), { 0, 20 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{ 0, 255, 0 });

        func(res);

        CameraFrame frame;
        frame.frame = std::move(res);

        sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(newKey, std::move(frame), name));
    }

    cv::Mat scaledResize(const cv::Mat& img) {
        float r = std::min(mConfig.inputWidth / (img.cols * 1.0), mConfig.inputHeight / (img.rows * 1.0));
        int unpadWidth = r * img.cols;
        int unpadHeight = r * img.rows;

        int dw = mConfig.inputWidth - unpadWidth;
        int dh = mConfig.inputHeight - unpadHeight;

        dw /= 2;
        dh /= 2;

        mTransformMatrix << 1.0 / r, 0, -dw / r, 0, 1.0 / r, -dh / r, 0, 0, 1;

        cv::Mat re;
        cv::resize(img, re, cv::Size(unpadWidth, unpadHeight));
        cv::Mat out;
        cv::copyMakeBorder(re, out, dh, dh, dw, dw, cv::BorderTypes::BORDER_CONSTANT);

        return out;
    }

    void blobImg(const cv::Mat& resizedImg, float* blobDataPtr) {
        cv::Mat pre;
        cv::Mat preSplit[3];
        resizedImg.convertTo(pre, CV_32F);
        cv::split(pre, preSplit);

        auto imgOffset = mConfig.inputHeight * mConfig.inputWidth;
        // 将img拷贝进blob
        for(int c = 0; c < 3; c++) {
//             memcpy(blobDataPtr, preSplit[c].data, imgOffset * sizeof(float));
            std::copy(reinterpret_cast<float*>(preSplit[c].data),
                      reinterpret_cast<float*>(preSplit[c].data) + imgOffset * sizeof(uchar), blobDataPtr);
            blobDataPtr += imgOffset;
        }
    }

    void generateGridsAndStride(int targetWidth, int targetHeight, std::vector<int>& strides,
                                std::vector<GridAndStride>& gridStrides) {
        for(auto stride : strides) {
            int numGridWidth = targetWidth / stride;
            int numGridHeight = targetHeight / stride;

            for(int g1 = 0; g1 < numGridHeight; g1++) {
                for(int g0 = 0; g0 < numGridWidth; g0++) {
                    gridStrides.push_back(GridAndStride{ g0, g1, stride });
                }
            }
        }
    }

    int argmax(const float* ptr, int len) {
        int maxArg = 0;
        for(int i = 1; i < len; i++) {
            if(ptr[i] > ptr[maxArg])
                maxArg = i;
        }
        return maxArg;
    }

    cv::Rect2i expandRect(const cv::Rect2f& rect, int oriWidth, int oriHeight) {
        constexpr float expandRatio = 1.3;
        float fx = rect.x + rect.width * (1 - expandRatio) / 2;
        float fy = rect.y + rect.height * (1 - expandRatio) / 2;
        int x = fx > 0 ? static_cast<int>(fx) : 0;
        int y = fy > 0 ? static_cast<int>(fy) : 0;
        int w = static_cast<int>(rect.width * expandRatio);
        int h = static_cast<int>(rect.height * expandRatio);
        w = x + w <= oriWidth ? w : oriWidth - x;
        h = y + h <= oriHeight ? h : oriHeight - y;
        return cv::Rect2i{ x, y, w, h };
    }

    void generateYoloxProposals(const std::vector<GridAndStride>& gridStrides, const float* featPtr, float probThreshold,
                                std::vector<Armor>& armors) {

        const int numAnchors = gridStrides.size();
        // Travel all the anchors
        for(int anchorIndex = 0; anchorIndex < numAnchors; anchorIndex++) {
            const int grid0 = gridStrides[anchorIndex].grid0;
            const int grid1 = gridStrides[anchorIndex].grid1;
            const int stride = gridStrides[anchorIndex].stride;

            // 9 means(x1, y1, x2, y2, x3, y3, x4, y4, confidence).size()
            const int basicPos = anchorIndex * (9 + mConfig.numColors + mConfig.numClasses);

            // yolox/models/yolo_head.py decode logic
            //  outputs[..., :2] = (outputs[..., :2] + grids) * strides
            //  outputs[..., 2:4] = torch.exp(outputs[..., 2:4]) * strides
            float x1 = (featPtr[basicPos + 0] + grid0) * stride;
            float y1 = (featPtr[basicPos + 1] + grid1) * stride;
            float x2 = (featPtr[basicPos + 2] + grid0) * stride;
            float y2 = (featPtr[basicPos + 3] + grid1) * stride;
            float x3 = (featPtr[basicPos + 4] + grid0) * stride;
            float y3 = (featPtr[basicPos + 5] + grid1) * stride;
            float x4 = (featPtr[basicPos + 6] + grid0) * stride;
            float y4 = (featPtr[basicPos + 7] + grid1) * stride;

            int boxColor = argmax(featPtr + basicPos + 9, mConfig.numColors);
            int boxClass = argmax(featPtr + basicPos + 9 + mConfig.numColors, mConfig.numClasses);

            float boxProb = (featPtr[basicPos + 8]);

            if(boxProb >= probThreshold) {
                Armor armor;

                Eigen::Matrix<float, 3, 4> light4PointNorm;
                Eigen::Matrix<float, 3, 4> light4PointDst;

                light4PointNorm << x1, x2, x3, x4, y1, y2, y3, y4, 1, 1, 1, 1;

                light4PointDst = mTransformMatrix * light4PointNorm;

                armor.light4Point.resize(4);
                for(int i = 0; i < 4; i++) {
                    armor.light4Point[i] = cv::Point2f(light4PointDst(0, i), light4PointDst(1, i));
                }

                std::vector<cv::Point2f> tmp(armor.light4Point.data(), armor.light4Point.data() + 4);
                armor.lightRect = cv::boundingRect(tmp);

                armor.robotType = boxClass;
                armor.robotColor = static_cast<Color>(boxColor);
                armor.prob = boxProb;

                armors.push_back(armor);
            }
        }  // point anchor loop
    }

    float intersectionArea(const Armor& a, const Armor& b) {
        cv::Rect2f inter = a.lightRect & b.lightRect;
        return inter.area();
    }

    void nmsSortedBboxes(std::vector<Armor>& faceObjects, std::vector<int>& picked, float nmsThreshold) {
        picked.clear();

        const int n = faceObjects.size();

        std::vector<float> areas(n);
        for(int i = 0; i < n; i++) {
            areas[i] = faceObjects[i].lightRect.area();
        }

        for(int i = 0; i < n; i++) {
            Armor& a = faceObjects[i];

            bool keep = true;
            for(uint32_t j = 0; j < picked.size(); j++) {
                Armor& b = faceObjects[picked[j]];

                // intersection over union
                float interArea = intersectionArea(a, b);
                float unionArea = areas[i] + areas[picked[j]] - interArea;
                float iou = interArea / unionArea;
                if(iou > nmsThreshold) {
                    keep = false;
                }
            }

            if(keep)
                picked.push_back(i);
        }
    }

    void decodeOutputs(const float* prob, std::vector<Armor>& armors) {
        std::vector<Armor> proposals;
        std::vector<int> strides = { 8, 16, 32 };
        std::vector<GridAndStride> gridStrides;

        generateGridsAndStride(mConfig.inputWidth, mConfig.inputHeight, strides, gridStrides);
        generateYoloxProposals(gridStrides, prob, mConfig.bboxConfThresh, proposals);

        std::sort(proposals.begin(), proposals.end(), [](const auto& lhs, const auto& rhs) { return lhs.prob > rhs.prob; });

        if(proposals.size() >= mConfig.topK)
            proposals.resize(mConfig.topK);
        std::vector<int> picked;
        nmsSortedBboxes(proposals, picked, mConfig.nmsThresh);
        int count = picked.size();
        armors.resize(count);

        for(int i = 0; i < count; i++) {
            armors[i] = proposals[picked[i]];
        }
    }

    std::vector<cv::Point2f> extractLightPoint(const cv::Mat& binary) {
        std::vector<std::vector<cv::Point2i>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<cv::RotatedRect> lights;
        for(auto& lightContour : contours) {
            cv::RotatedRect lightRect;
            if(lightContour.size() < 6) {
                continue;
            }
            lightRect = cv::minAreaRect(lightContour);
            lights.emplace_back(lightRect);
        }
        if(lights.size() < 2) {
            return {};
        }

        // 找到最左边和最右边的两个灯条
        int lLight = 0, rLight = lights.size() - 1;
        for(uint32_t i = 1; i < lights.size(); ++i) {
            if(lights[i].center.x < lights[lLight].center.x) {
                lLight = i;
            }
            if(lights[i].center.x > lights[rLight].center.x) {
                rLight = i;
            }
        }

        const auto clcTopAndBottom = [](const cv::RotatedRect& lightRect) {
            cv::Point2f p[4];
            lightRect.points(p);
            std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
            auto top = (p[0] + p[1]) / 2;
            auto bottom = (p[2] + p[3]) / 2;
            return std::pair{ top, bottom };
        };
        auto [lt, lb] = clcTopAndBottom(lights[lLight]);
        auto [rt, rb] = clcTopAndBottom(lights[rLight]);
        return std::vector<cv::Point2f>{ lt, lb, rb, rt };
    }

    std::vector<Armor> postProcess(const std::vector<Armor>& armors, const cv::Mat& img) {
        std::vector<Armor> enemyArmors;

        for(const auto& armor : armors) {
            if(armor.robotColor == GlobalSettings::get().getColor() || armor.robotColor == Color::Negative)
                continue;

            // 采用传统视觉提取角点，提高pnp精度
            Armor enemyArmor = armor;
            cv::Mat roiArmor, gray, binary;
            auto roiArmorRect = expandRect(armor.lightRect, img.cols, img.rows);
            roiArmor = img(roiArmorRect);
            cv::cvtColor(roiArmor, gray, cv::COLOR_BGR2GRAY);
            cv::threshold(gray, binary, mConfig.binaryThresh, 255, cv::THRESH_BINARY);
            if(mConfig.debugView) {
                debugView("roiArmor", roiArmor, [](auto) {});
                debugView("binary", binary, [](auto) {});
            }
            auto light4Points = extractLightPoint(binary);
            if(light4Points.size() == 4) {
                auto tlOffset = roiArmorRect.tl();
                for(int i = 0; i < 4; ++i) {
                    enemyArmor.light4Point[i] = { light4Points[i].x + tlOffset.x, light4Points[i].y + tlOffset.y };
                }
            }

            enemyArmors.push_back(enemyArmor);
        }
        return enemyArmors;
    }

public:
    NNetArmorDetector(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        // 1. 读取网络
        mNetwork = mIe.ReadNetwork(mConfig.networkPath);
        if(mNetwork.getOutputsInfo().size() != 1) {
            throw std::logic_error("Sample supports topologies with 1 output only");
        }

        // 2. 配置输入输出blob
        // 输入blob
        auto [inputName, inputInfo] = *mNetwork.getInputsInfo().begin();

        auto [outputName, outputInfo] = *mNetwork.getOutputsInfo().begin();

        // 3. 加载网络
        mExeNetwork = mIe.LoadNetwork(mNetwork, "CPU");

        // 4. 创建推理请求
        mInferRequest = mExeNetwork.CreateInferRequest();

        const auto inputBlob = mInferRequest.GetBlob(inputName);
        mInputMemBlobPtr = IE::as<IE::MemoryBlob>(inputBlob);

        const IE::Blob::Ptr outputBlob = mInferRequest.GetBlob(outputName);
        mOutputMemBlobPtr = IE::as<IE::MemoryBlob>(outputBlob);
    }

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](image_frame_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame, std::string_view>);
                      ACTOR_EXCEPTION_PROBE();

                     const auto t0 = Clock::now();
                     const auto frame = std::get<0>(BlackBoard::instance().get<CameraFrame, std::string_view>(key).value());
                     DetectedArmorArray res;
                     res.frame = frame;

                     auto croppedImg = scaledResize(res.frame.frame);

                     auto inputBlobHolder = mInputMemBlobPtr->wmap();
                     float* blobDataPtr = inputBlobHolder.as<float*>();
                     blobImg(croppedImg, blobDataPtr);

                     mInferRequest.Infer();

                     std::vector<Armor> allArmors;
                     auto outputHolder = mOutputMemBlobPtr->rmap();
                     const float* netPredict = outputHolder.as<const IE::PrecisionTrait<IE::Precision::FP32>::value_type*>();
                     decodeOutputs(netPredict, allArmors);

                     res.armors = postProcess(allArmors, res.frame.frame);

                     const auto t2 = Clock::now();
                     logInfo(
                         fmt::format("NNet armor detector:decode time {:.4f}ms", static_cast<double>((t2 - t0).count()) / 1e6));

                     sendAll(armor_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 } };
    }
};

HUB_REGISTER_CLASS(NNetArmorDetector);
