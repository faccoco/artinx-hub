#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedCar.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <string>
#include <utility>

struct ArmorDetectorSettings final {
    // Settings issuing color extracting. 颜色提取。
    int enemyColor;
    std::vector<std::string> colorName;
    std::vector<int> colorRangeLeft;
    std::vector<int> colorRangeRight;
    std::vector<bool> colorRangeComplement;
    // Settings issuing suitable armor. 装甲板识别
    int colorThreshold;   // color threshold for colorImg from substract channels 通道相减的colorImg使用的二值化阈值
    int brightThreshold;  // color threshold for brightImg 亮度图二值化阈值
    float minArea;        // min area of light bar 灯条允许的最小面积
    float maxArea;        // min area of light bar 灯条允许的最小面积
    float maxAngle;       // max angle of light bar 灯条允许的最大偏角
    float maxAngleDiff;  // max angle difference between two light bars 两个灯条之间允许的最大角度差
    float maxLengthDiffRatio;  // max length ratio difference between two light bars 两个灯条之间允许的最大长度差比值
    float maxDeviationAngle;  // max deviation angle 两灯条最大错位角
    float maxYDiffRatio;     // max y
    float maxXDiffRatio;     // max x
    /// (lzj)
    float minXDiffRatio;
};

template <class Inspector>
bool inspect(Inspector& f, ArmorDetectorSettings& x) {
    return f.object(x).fields(
        f.field("enemyColor", x.enemyColor), f.field("colorName", x.colorName),
        f.field("colorRangeLeft", x.colorRangeLeft), f.field("colorRangeRight", x.colorRangeRight),
        f.field("colorRangeComplement", x.colorRangeComplement), f.field("colorThreshold", x.colorThreshold),
        f.field("brightThreshold", x.brightThreshold), f.field("minArea", x.minArea), f.field("maxArea", x.maxArea),
        f.field("maxAngle", x.maxAngle), f.field("maxAngleDiff", x.maxAngleDiff),
        f.field("maxLengthDiffRatio", x.maxLengthDiffRatio), f.field("maxDeviationAngle", x.maxDeviationAngle),
        f.field("maxYDiffRatio", x.maxYDiffRatio), f.field("maxXDiffRatio", x.maxXDiffRatio),
        f.field("minXDiffRatio", x.minXDiffRatio));
}

#define DEBUG_BY_TESTER  // defined by 叶璨铭, for detector to explain why not detect an armor.

class ArmorDetector final : public HubHelper<caf::event_based_actor, ArmorDetectorSettings, armor_detect_available_atom> {
    Identifier mKey;
#ifdef _DEBUG
#ifdef DEBUG_BY_TESTER
    size_t testingCount = 0;  //被tester测试时，尝试获得当前被测试的图片的编号
#else
    size_t mBoxFrameCnt = 0;  //上场debug的时候，200帧输出一次判断的解释
#endif
#endif
    std::vector<PairedLight> solve(const cv::Mat& image) {
        using std::cout;
        using std::endl;
        // Color classification is required.
        const auto monoImage = extractColor(image);
#ifdef _DEBUG
#ifdef DEBUG_BY_TESTER
        std::stringstream ss;
        testingCount++;
        assert(monoImage.type() == CV_8U);
#endif
#endif
        auto lights = findLights(monoImage);
        if(lights.empty()) {
            CAF_LOG_INFO("light not found.");
            return {};
        }
#ifdef _DEBUG
        ss << lights.size() << " lights found." << endl;
#endif
        auto result = matchLights(lights);
#ifdef _DEBUG
        if(!result.empty()) {
            ss << "succeeded at " << testingCount << endl;
        }
        CAF_LOG_INFO(ss.str());
#endif
        return result;
    }

    cv::Mat extractColor(const cv::Mat& srcImage) {
        cv::Mat srcImage_HSV, red_bin_1, red_bin_2;
        cv::Mat result = cv::Mat::zeros(srcImage.size(), CV_8UC1);
        cv::cvtColor(srcImage, srcImage_HSV, cv::COLOR_BGR2HSV);
        int bl_lw_s = 170;
        int bl_lw_v = 100;
        int rd_lw_s = 180;
        int rd_lw_v = 100;
        if(mConfig.colorRangeComplement[mConfig.enemyColor]) {
            /// (lzj) red: 0~10, 156~180
            cv::inRange(srcImage_HSV, cv::Scalar(0, rd_lw_s, rd_lw_v),
                        cv::Scalar(mConfig.colorRangeLeft[mConfig.enemyColor], 255, 255), red_bin_1);
            cv::inRange(srcImage_HSV, cv::Scalar(mConfig.colorRangeRight[mConfig.enemyColor], rd_lw_s, rd_lw_v),
                        cv::Scalar(180, 255, 255), red_bin_2);
            cv::bitwise_or(red_bin_1, red_bin_2, result);
        } else {
            /// (lzj) blue: 100~124
            cv::inRange(srcImage_HSV, cv::Scalar(mConfig.colorRangeLeft[mConfig.enemyColor], bl_lw_s, bl_lw_v),
                        cv::Scalar(mConfig.colorRangeRight[mConfig.enemyColor], 255, 255), result);
        }
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        dilate(result, result, kernel);  // dilate the roiImg_binary which can make the lightBar area more smooth
        // 对roiIng_binary进行膨胀操作，使得灯条区域更加平滑有衔接

        return result;
    }

    std::vector<cv::RotatedRect> findLights(const cv::Mat& image) {
        using namespace std;
        // 1.find Contours. 找轮廓。不需要嵌套层次的轮廓。可以只要拐点。不需要偏移量。
        vector<vector<cv::Point2i>> lightContours;
        cv::findContours(image, lightContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        // 2.every contour(set of points) may form a light bar.每一个轮廓形成一个灯条
        vector<cv::RotatedRect> lights;
        for(const auto& lightContour : lightContours) {
            if(lightContour.size() < 6)
                continue;
            auto lightRect = cv::fitEllipse(lightContour);
            // 2.filter suitable contour. 筛选
            auto areaInt = lightRect.boundingRect().area();//FIXME: calculate area two times may be slow.
            auto areaFloat = cv::contourArea(lightContour);//FIXME: 没调查清楚，能不能统一用areaInt去判断。目前先按照原本代码的逻辑来。
            if(areaFloat < mConfig.minArea || (mConfig.maxArea < areaInt && lightRect.size.width > lightRect.size.height*0.6))
                continue;
            if(abs(lightRect.angle) > mConfig.maxAngle)
                continue;
            if(lightRect.size.width > lightRect.size.height * 1.3f)
                continue;
            lights.emplace_back(lightRect);
        }
        return lights;  // return by moving constructor. 移动构造返回
    }

    std::vector<PairedLight> matchLights(std::vector<cv::RotatedRect>& lights) {
        using namespace std;
        if(lights.size() < 2)
            return {};  // A single light can never construct an armor. 一个巴掌拍不响。
        sort(lights.begin(), lights.end(),
             [](const cv::RotatedRect& rr1, const cv::RotatedRect& rr2) { return rr1.center.x < rr2.center.x; });
        std::vector<IndexedPairedLight> armors;
        for(size_t i = 0; i < lights.size() - 1; i++) {
            for(size_t j = i + 1; j < lights.size();
                j++)  // just ensure every two lights be matched once 从左至右，每个灯条与其他灯条一次匹配判断
            {
                IndexedPairedLight armor =
                    IndexedPairedLight{ PairedLight{ lights[i], lights[j] }, i,
                                        j };  // construct an armor using the matchable lights 利用左右灯条构建装甲板
                if(isSuitableArmor(armor.data))
                // when the armor we constructed just now is a suitable one,set extra information of
                {
                    armors.emplace_back(armor);  // push into armors 将匹配好的装甲板push入armors中
                }
#ifdef _DEBUG
                std::stringstream ss;
                ss << "At picture" << testingCount << " failed." << std::endl;
                ss << "The " << i << " and the " << j << " light cannot form a suitable armor." << std::endl;
                CAF_LOG_INFO(ss.str());
#endif
            }
            eraseErrorRepeatArmor(armors);  // delete the error armor caused by error light 删除游离灯条导致的错误装甲板
        }
        std::vector<PairedLight> pairedLights;
        for(const auto& armor : armors) {
            pairedLights.emplace_back(armor.data);
        }
        return pairedLights;
    }

    struct IndexedPairedLight {
        PairedLight data;
        size_t index1;
        size_t index2;
        IndexedPairedLight(PairedLight data, size_t index1, size_t index2) : data(std::move(data)), index1(index1), index2(index2) {}
    };

    inline bool isSuitableArmor(const PairedLight& armor) {
        using std::vector;
        vector<bool> conditions;
        conditions.push_back(getAngleDiff(armor) <
                             mConfig.maxAngleDiff + 3);  //+10 max,  real uac may be higher than you think(ycm changed)
        // angle difference judge the angleDiff should be less than maxAngleDiff 灯条角度差判断，需小于允许的最大角差
        conditions.push_back(getDeviationAngle(armor) < mConfig.maxDeviationAngle);
        // deviation angle judge: the horizon angle of the line of centers of lights 灯条错位度角(两灯条中心连线与水平线夹角)判断
        conditions.push_back(getDislocationX(armor) < mConfig.maxXDiffRatio);
        // dislocation judge: the x and y can not be too far 灯条位置差距 两灯条中心x、y方向差距不可偏大（用比值作为衡量依据）
        conditions.push_back(getDislocationY(armor) < mConfig.maxYDiffRatio + 0.1);
        // dislocation judge: the x and y can not be too far 灯条位置差距 两灯条中心x、y方向差距不可偏大（用比值作为衡量依据）
        conditions.push_back(getLengthRatio(armor) < mConfig.maxLengthDiffRatio);
#ifdef _DEBUG
#ifdef DEBUG_BY_TESTER
        std::stringstream ss;
        int i = 0;
        vector<std::string> messages;
        messages.push_back("灯条角度差大于允许的最大角差!");
        messages.push_back("灯条错位度角(两灯条中心连线与水平线夹角)过大!");
        messages.push_back("灯条位置差距 两灯条中心x方向差距偏大（用比值作为衡量依据）！");
        messages.push_back("灯条位置差距 两灯条中心y方向差距偏大（用比值作为衡量依据）！");
        messages.push_back("左右灯条长度差比值过大！");
        bool success = true;
        for(const auto& condition : conditions) {
            if(!condition) {
                ss << "Condition" << i << " not satisfied. Error message for that condition is " << messages[i] << std::endl;
                success = false;
            }
            i++;
        }
        if(!success) {
            ss << "At picture" << testingCount << " failed." << std::endl;
        }
        CAF_LOG_INFO(ss.str());
#else
        if((++mBoxFrameCnt) % 200 == 0) {
            std::stringstream ss;
            ss << "angle difference judge; "
               << "deviation angle judge; "
               << "dislocation judge:x; "
               << "dislocation judge:y; "
               << "length difference ration judge; " << std::endl;
            ss << condition1 << condition2 << condition3 << condition4 << condition5 << std::endl;
            CAF_LOG_INFO(ss.str());
        }
#endif
#endif
        for(const auto& condition : conditions)
            if(!condition)
                return false;
        return true;
    }

    static void eraseErrorRepeatArmor(std::vector<IndexedPairedLight>& armors) {
        const size_t length = armors.size();
        const auto it = armors.begin();
        for(size_t i = 0; i < length; i++)
            for(size_t j = i + 1; j < length; j++) {
                if(armors[i].index1 == armors[j].index1 || armors[i].index1 == armors[j].index2 ||
                   armors[i].index2 == armors[j].index1 || armors[i].index2 == armors[j].index2) {
                    if(getDeviationAngle(armors[i].data) > getDeviationAngle(armors[j].data)) {
                        armors.erase(it + i);
                    } else {
                        armors.erase(it + j);
                    }
                }
            }
    }

    // angle difference: the angle difference of left and right lights 装甲板左右灯条角度差
    static inline float getAngleDiff(const PairedLight& armor) {
        const float angleDiff = abs(armor.r1.angle - armor.r2.angle);  // get the abs of angle_diff 灯条的角度差
        return angleDiff;
    }

    // deviation angle : the horizon angle of the line of centers of lights 灯条错位度角(两灯条中心连线与水平线夹角)
    static inline float getDeviationAngle(const PairedLight& armor) {
        const float deltaX = armor.r2.center.x - armor.r1.center.x;                                         //Δx
        const float deltaY = armor.r2.center.y - armor.r1.center.y;                                         //Δy
        const float deviationAngle = 180.0f * std::abs(atan(deltaY / deltaX)) / static_cast<float>(CV_PI);  // tanθ=Δy/Δx
        return deviationAngle;
    }

    static inline float lightLength(const cv::RotatedRect& light) {
        return std::max(light.size.height, light.size.width);
    }

    // dislocation judge X: r-l light center distance ration on the X-axis 灯条位置差距 两灯条中心x方向差距比值
    static inline float getDislocationX(const PairedLight& armor) {
        const float meanLen = (lightLength(armor.r1) + lightLength(armor.r2)) / 2;
        const float xDiff =
            std::abs(armor.r1.center.x - armor.r2.center.x);  // x distance ration x轴方向上的距离比值（y轴距离与灯条平均值的比）
        const float xDiffRatio = xDiff / meanLen;
        return xDiffRatio;
    }

    // dislocation judge Y: r-l light center distance ration on the Y-axis 灯条位置差距 两灯条中心y方向差距比值
    static inline float getDislocationY(const PairedLight& armor) {
        const float meanLen = (lightLength(armor.r1) + lightLength(armor.r2)) / 2;
        const float yDiff =
            std::abs(armor.r1.center.y - armor.r2.center.y);  // y distance ration y轴方向上的距离比值（x轴距离与灯条平均值的比）
        const float yDiffRatio = yDiff / meanLen;
        return yDiffRatio;
    }

    // length difference ration: the length difference ration r-l lights 左右灯条长度差比值
    static inline float getLengthRatio(const PairedLight& armor) {
        /// (lzj) match armor : use the smaller one's lengh instead of mean value of the two
        const auto leftArmorLength = lightLength(armor.r1);
        const auto rightArmorLength = lightLength(armor.r2);
        const float lengthDiff = std::abs(leftArmorLength - rightArmorLength);
        const float lengthDiffRatio = lengthDiff / std::min(leftArmorLength, rightArmorLength);
        return lengthDiffRatio;
    }

public:
    ArmorDetector(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(ArmorDetector).hash_code() } {}

    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](car_detect_available_atom, Identifier key) {
                     const auto data = BlackBoard::instance().get<DetectedCarArray>(key).value();

                     DetectedArmorArray res;
                     res.lastUpdate = data.frame.lastUpdate;
                     res.cameraInfo = data.frame.info;

                     for(auto& roi : data.cars) {
                         auto armors = solve(data.frame.frame(roi));
                         res.armors.push_back({ roi, 0, std::move(armors) });  // TODO: id
                     }

                     BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(armor_detect_available_atom_v, mKey);
                 } };
    }
};

HUB_REGISTER_CLASS(ArmorDetector);
