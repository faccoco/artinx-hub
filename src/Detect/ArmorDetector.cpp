#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedCar.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <fmt/format.h>
#include <magic_enum.hpp>
#include <string>
#include <utility>
enum class DetectorRunningType { RELEASE, DEBUG, DATASET_BASED_TEST };
struct ArmorDetectorSettings final {
    std::string runningTypeString;    // One of DetectorRunningType. Reading string from file.
    DetectorRunningType runningType;  // inspect enum value from string.
    // Settings issuing color extracting.
    int enemyColor;
    std::vector<std::string> colorName;
    std::vector<int> colorHueRangeLowerBound;
    std::vector<int> colorHueRangeUpperBound;
    std::vector<bool> colorHueRangeIsComplemented;
    // Settings issuing suitable armor.
    float minArea;             // min area of light bar.
    float maxArea;             // max area of light bar.
    float maxAngle;            // max angle of light bar.
    float maxAngleDiff;        // max angle difference between two light bars.
    float maxLengthDiffRatio;  // max length ratio difference between two light bars.
    float maxDeviationAngle;   // max deviation angle.
    float maxYDiffRatio;       // max light center distance ratio on the Y-axis
    float maxXDiffRatio;       // max light center distance ratio on the X-axis
    float minXDiffRatio;       // min light center distance ratio on the X-axis
};

template <class Inspector>
bool inspect(Inspector& f, ArmorDetectorSettings& x) {
    return f.object(x).fields(
        f.field("runningTypeString", x.runningTypeString).invariant([&x](const std::string& runningTypeString) {
            auto e = magic_enum::enum_cast<DetectorRunningType>(runningTypeString);
            if(e.has_value()) {
                x.runningType = e.value();
                return true;
            }
            return false;
        }),
        f.field("enemyColor", x.enemyColor), f.field("colorName", x.colorName),
        f.field("colorHueRangeLowerBound", x.colorHueRangeLowerBound),
        f.field("colorHueRangeUpperBound", x.colorHueRangeUpperBound),
        f.field("colorHueRangeIsComplemented", x.colorHueRangeIsComplemented), f.field("minArea", x.minArea),
        f.field("maxArea", x.maxArea), f.field("maxAngle", x.maxAngle), f.field("maxAngleDiff", x.maxAngleDiff),
        f.field("maxLengthDiffRatio", x.maxLengthDiffRatio), f.field("maxDeviationAngle", x.maxDeviationAngle),
        f.field("maxYDiffRatio", x.maxYDiffRatio), f.field("maxXDiffRatio", x.maxXDiffRatio),
        f.field("minXDiffRatio", x.minXDiffRatio));
}

class ArmorDetector final : public HubHelper<caf::event_based_actor, ArmorDetectorSettings, armor_detect_available_atom> {
    Identifier mKey;
    size_t mFrameCnt = 0;
    std::vector<PairedLight> solve(const cv::Mat& image) {
        // Color classification is required.
        const auto monoImage = extractColor(image);
        switch(mConfig.runningType) {
            case DetectorRunningType::RELEASE: {
                auto lights = findLights(monoImage);
                auto result = matchLights(lights);
                return result;
            }
            case DetectorRunningType::DATASET_BASED_TEST:
                mFrameCnt++;
            case DetectorRunningType::DEBUG: {  // includes DATASET_BASED_TEST branch
                if(monoImage.type() != CV_8U)
                    throw std::runtime_error("mono image not cv_8u");
                auto lights = findLights(monoImage);
                if(lights.empty()) {
                    CAF_LOG_INFO("light not found.");
                    return {};
                }
                CAF_LOG_INFO(fmt::format("{} lights found.", lights.size()));
                auto result = matchLights(lights);
                if(!result.empty()) {
                    CAF_LOG_INFO(fmt::format("succeeded at {}", mFrameCnt));
                }
                return result;
            }
        }
    }

    cv::Mat extractColor(const cv::Mat& srcImage) const {
        cv::Mat srcImage_HSV;
        cv::Mat result = cv::Mat::zeros(srcImage.size(), CV_8UC1);
        cv::cvtColor(srcImage, srcImage_HSV, cv::COLOR_BGR2HSV);
        constexpr int blueLowestSaturation = 170;
        constexpr int blueHighestSaturation = 255;
        constexpr int blueLowestVue = 46;
        constexpr int blueHighestVue = 255;
        constexpr int redLowestSaturation = 180;  // typically 46, but we may need brighter red.
        constexpr int redHighestSaturation = 255;
        constexpr int redLowestVue = 100;  // typically 46, but we may need brighter red.
        constexpr int redHighestVue = 255;
        if(mConfig.colorHueRangeIsComplemented[mConfig.enemyColor]) {
            cv::Mat redBinary1, redBinary2;
            // red like colors have two ranges, for example, red hue range: [0,10], [156,180]. Notice that opencv hsv's h range is
            // [0,180].
            // 1.the first range. inRange function can check the first argument, and binarize the according to range and save to
            // the last argument dst.
            cv::inRange(srcImage_HSV, cv::Scalar(0, redLowestSaturation, redLowestVue),
                        cv::Scalar(mConfig.colorHueRangeLowerBound[mConfig.enemyColor], redHighestSaturation, redHighestVue),
                        redBinary1);
            // 2.the second range.
            cv::inRange(srcImage_HSV,
                        cv::Scalar(mConfig.colorHueRangeUpperBound[mConfig.enemyColor], redLowestSaturation, redLowestVue),
                        cv::Scalar(180, redHighestSaturation, redHighestVue), redBinary2);
            cv::bitwise_or(redBinary1, redBinary2, result);  // dst serves as a mask. In this case, we have two range ,and we
                                                             // combine them together, so we used bitwise or to combine two masks.
        } else {
            // blue like colors have only one range, for example, blue hue range: [100,124]
            cv::inRange(srcImage_HSV,
                        cv::Scalar(mConfig.colorHueRangeLowerBound[mConfig.enemyColor], blueLowestSaturation, blueLowestVue),
                        cv::Scalar(mConfig.colorHueRangeUpperBound[mConfig.enemyColor], blueHighestSaturation, blueHighestVue),
                        result);  // simply let the binarized image to be the result.
        }
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        dilate(result, result, kernel);  // dilate the result which can make the lightBar area more smooth
        return result;
    }

    std::vector<cv::RotatedRect> findLights(const cv::Mat& image) const {
        // 1.find Contours.
        std::vector<std::vector<cv::Point2i>> lightContours;
        cv::findContours(image, lightContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        // 2.every contour(set of points) may form a light bar.
        std::vector<cv::RotatedRect> lights;
        for(const auto& lightContour : lightContours) {
            if(lightContour.size() < 6)  // points are too few to form a light bar.
                continue;
            auto lightRect = cv::fitEllipse(lightContour);
            // 2.filter suitable contour.
            // 2.1 the light bar may be too small or too big.
            const double area = relativeArea(lightRect, image);
            if(area < mConfig.minArea || (mConfig.maxArea < area && lightRect.size.width > lightRect.size.height * 0.6))
                continue;
            // 2.2 the light bar may be too inclined.
            if(std::fabs(lightRect.angle) > mConfig.maxAngle)
                continue;

            // 2.3 the light bar may be too wide, while it is suspected to be slim and tall.
            constexpr double maxWidthHeightRatio = widthOfLightBar / heightOfLightBar *
                9;  // typically, it is 0.147, but the armor dilates and may be very fat. So I prefer to allow 9 times Ratio.
                    // FIXME: the evidence of 9 times but not 3 times or 10 times is not sound, it is just guessed. need
                    // experiment or proof.
            if(lightRect.size.width > lightRect.size.height * maxWidthHeightRatio)
                continue;
            lights.emplace_back(lightRect);
        }
        return lights;  // return by moving construction.
    }
    static double relativeArea(const cv::RotatedRect& lightRect, const cv::Mat& image) {
        const double area = lightRect.size.height *
            lightRect.size.width;  // this is the correct area calculation.
                                   //  It's the minAreaBoundingRotatedRectangle of the contour, but not the bounding
                                   //  normal(0°) rectangle of the minAreaBoundingRotatedRectangle.
        constexpr double configAssumedImageArea =
            1280 * 1024;  // config experimental deciding are based on the case when camera is         constexpr double
                          // configAssumedImageArea = 1280 * 1024;//config experimental deciding are based on the case when camera
                          // and full image is the roi.
        const double imageArea = static_cast<double>(image.rows) *
            static_cast<double>(image.cols);  // use double because the calculation may require higher precision to be correct.
        return area * configAssumedImageArea /
            imageArea;  // we want to transform the current area in the picture to be the relative area in the configAssumedImage,
                        // so that we can compare it with the config areas.
    }

    std::vector<PairedLight> matchLights(std::vector<cv::RotatedRect>& lights) {
        if(lights.size() < 2)
            return {};  // A single light can not construct an armor.
        std::sort(lights.begin(), lights.end(),
                  [](const cv::RotatedRect& rr1, const cv::RotatedRect& rr2) { return rr1.center.x < rr2.center.x; });
        std::vector<IndexedPairedLight> armors;
        for(size_t i = 0; i < lights.size() - 1; i++) {
            for(size_t j = i + 1; j < lights.size(); j++)  // just ensure every two lights are matched once and only once.
            {
                auto armor = IndexedPairedLight{
                    PairedLight{ lights[i], lights[j] }, i, j
                };  // construct an armor using the matchable lights. add the index information for later usages.
                if(isSuitableArmor(armor.data)) {
                    armors.emplace_back(armor);  // when the armor we constructed just now is a suitable one, just push it back.
                }
                if(mConfig.runningType == DetectorRunningType::DATASET_BASED_TEST) {
                    CAF_LOG_INFO(fmt::format("At picture {} failed.\nThe {}'s and the {}'s light cannot form a suitable armor.",
                                             mFrameCnt, i, j));
                }
            }
            eraseErrorRepeatArmor(armors);  // delete the error armor caused by error light
        }
        std::vector<PairedLight> pairedLights;
        for(const auto& armor : armors) {
            pairedLights.emplace_back(armor.data);
        }
        return pairedLights;
    }

    struct IndexedPairedLight final {
        PairedLight data;
        size_t index1;
        size_t index2;
    };

    bool isSuitableArmor(const PairedLight& armor) {
        bool conditions[5] = {
            (std::fabs(armor.r1.angle - armor.r2.angle) <
             mConfig.maxAngleDiff),  // angle difference judge the angleDiff should be less than maxAngleDiff
            (getDeviationAngle(armor) <
             mConfig.maxDeviationAngle),  // deviation angle judge: the horizon angle of the line of centers of lights
            (getDislocationX(armor) < mConfig.maxXDiffRatio), // dislocation judge: the x and y can not be too far
            (getDislocationY(armor) < mConfig.maxYDiffRatio + 0.1), // dislocation judge: the x and y can not be too far
            (getLengthRatio(armor) < mConfig.maxLengthDiffRatio)   // length difference judge: the x and y should have similar length.
        };
        auto tempRunningType = mConfig.runningType;
        if(tempRunningType ==
           DetectorRunningType::DEBUG) {  // In debug mode, the image comes from camera, we put the armor
                                          // before camera and test whether the config works. So we print the log every 200 frame.
            if((++mFrameCnt) % 200 == 0)
                tempRunningType = DetectorRunningType::DATASET_BASED_TEST;
            else
                tempRunningType = DetectorRunningType::RELEASE;
        }
        switch(tempRunningType) {
            case DetectorRunningType::RELEASE: {
                for(const auto& condition : conditions)
                    if(!condition)
                        return false;
                return true;
            }
            case DetectorRunningType::DATASET_BASED_TEST: {  // includes DATASET_BASED_TEST branch
                std::string messages[5] = {
                    "Angle difference is now bigger than allowed max angle difference!",
                    "The horizon angle of the line of centers of lights is too big!",
                    "light center distance ratio on the X-axis between the two lights is too far!",
                    "light center distance ratio on the Y-axis between the two lights is too far!",
                    "the length difference ratio is too big!"
                };
                bool success = true;
                int i = 0;
                for(const auto& condition : conditions) {
                    if(!condition) {
                        CAF_LOG_INFO(fmt::format("Armor not suitable At picture {}\nCondition {} not satisfied. Error message "
                                                 "for that condition is \"{}\"",
                                                 mFrameCnt, i, messages[i]));
                        success = false;
                    }
                    i++;
                }
                return success;
            }
        }
    }
    // delete the error armor caused by error light
    static void eraseErrorRepeatArmor(std::vector<IndexedPairedLight>& armors) {
        std::vector<IndexedPairedLight> result;
        const size_t length = armors.size();
        for(size_t i = 0; i < length; i++)
            for(size_t j = i + 1; j < length; j++) {
                if(armors[i].index1 == armors[j].index1 || armors[i].index1 == armors[j].index2 ||
                   armors[i].index2 == armors[j].index1 || armors[i].index2 == armors[j].index2) {
                    if(getDeviationAngle(armors[i].data) > getDeviationAngle(armors[j].data)) {
                        result.emplace_back(armors[j]);  // do not use iterator to erase the original vector, otherwise the
                                                         // location i, j is wrong for next loop.
                    } else {
                        result.emplace_back(armors[i]);
                    }
                }
            }
        armors = std::move(result);  // Use result to moving construct armors again. (it clears armors first.)
    }

    // deviation angle : the horizon angle of the line of centers of lights
    static float getDeviationAngle(const PairedLight& armor) {
        const float deltaX = armor.r2.center.x - armor.r1.center.x;                                 //Δx
        const float deltaY = armor.r2.center.y - armor.r1.center.y;                                 //Δy
        const float deviationAngle = 180.0f * std::fabs(atan(deltaY / deltaX)) / glm::pi<float>();  // tanθ=Δy/Δx
        return deviationAngle;
    }

    // widely used in other getXXX functions
    static float lightLength(const cv::RotatedRect& light) {
        return std::fmax(light.size.height, light.size.width);
    }

    // dislocation judge X: right-left light center distance ratio on the X-axis
    static float getDislocationX(const PairedLight& armor) {
        const float meanLen = (lightLength(armor.r1) + lightLength(armor.r2)) / 2;
        const float xDiff = std::fabs(armor.r1.center.x - armor.r2.center.x);  // x distance ratio
        const float xDiffRatio = xDiff / meanLen;
        return xDiffRatio;
    }

    // dislocation judge Y: r-l light center distance ration on the Y-axis
    static float getDislocationY(const PairedLight& armor) {
        const float meanLen = (lightLength(armor.r1) + lightLength(armor.r2)) / 2;
        const float yDiff = std::fabs(armor.r1.center.y - armor.r2.center.y);  // y distance ratio
        const float yDiffRatio = yDiff / meanLen;
        return yDiffRatio;
    }

    // length difference ratio: the length difference ratio r-l lights
    static float getLengthRatio(const PairedLight& armor) {
        /// (lzj) match armor : use the smaller one's length instead of mean value of the two
        const auto leftArmorLength = lightLength(armor.r1);
        const auto rightArmorLength = lightLength(armor.r2);
        const float lengthDiff = std::fabs(leftArmorLength - rightArmorLength);
        const float lengthDiffRatio = lengthDiff / std::fmin(leftArmorLength, rightArmorLength);
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
                 } 
        };
    }
};

HUB_REGISTER_CLASS(ArmorDetector);
