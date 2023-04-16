#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedCar.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <cstdint>

#include <string>
#include <utility>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"

struct ArmorDetectorSettings final {
    float globalScale;
    double binaryThresh;
    std::vector<int32_t> thresholdForBlue;  // minBlue, maxGreen, maxRed
    std::vector<int32_t> thresholdForRed;   // minRed, maxBlue,maxGreen
    int bgrSubtractForBlue;                 // bgr subtract threshold for blue
    int bgrSubtractForRed;                  // bgr subtract threshold for red
    float minLightRectRatio;                // width/height
    float maxLightRectRatio;                // width/height
    float maxLightAngle;                    // angle(degree)
    float maxAreaRatio;                     // ellipseArea/contourArea
    float minArmorRectRatio;                // height/width
    float maxArmorRectRatio;                // height/width
    float maxArmorAngle;                    // angle(degree)
    float maxLightBaseAngle;                // abs(lightAngle-armorAngle)(degree)
    float maxParallelAngle;                 // abs(lightAngle1-lightAngle2)(degree)
    float minLightHeightRatio;              // lightHeight/armorHeight
};

template <class Inspector>
bool inspect(Inspector& f, ArmorDetectorSettings& x) {
    return f.object(x).fields(f.field("globalScale", x.globalScale),
                              f.field("thresholdForBlue", x.thresholdForBlue).invariant([](auto& c) { return c.size() == 3; }),
                              f.field("thresholdForRed", x.thresholdForRed).invariant([](auto& c) { return c.size() == 3; }),
                              f.field("bgrSubtractForBlue", x.bgrSubtractForBlue).fallback(60),
                              f.field("bgrSubtractForRed", x.bgrSubtractForRed).fallback(60),
                              f.field("minLightRectRatio", x.minLightRectRatio),
                              f.field("maxLightRectRatio", x.maxLightRectRatio), f.field("maxLightAngle", x.maxLightAngle),
                              f.field("maxAreaRatio", x.maxAreaRatio), f.field("minArmorRectRatio", x.minArmorRectRatio),
                              f.field("maxArmorRectRatio", x.maxArmorRectRatio), f.field("maxArmorAngle", x.maxArmorAngle),
                              f.field("maxLightBaseAngle", x.maxLightBaseAngle), f.field("maxParallelAngle", x.maxParallelAngle),
                              f.field("minLightHeightRatio", x.minLightHeightRatio));
}

class ArmorDetector final
    : public HubHelper<caf::event_based_actor, ArmorDetectorSettings, armor_detect_available_atom, image_frame_atom> {
    Identifier mKey;

    // TODO: light pairs affinity

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

    cv::Rect2f boundingRect(const PairedLight& armor) {
        const auto b1 = armor.r1.boundingRect2f();
        const auto b2 = armor.r2.boundingRect2f();
        return b1 | b2;
    }

    std::optional<Light> isLight(const cv::RotatedRect& lightRect) {
        Light light;
        cv::Point2f p[4];
        lightRect.points(p);
        std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
        light.top = (p[0] + p[1]) / 2;
        light.bottom = (p[2] + p[3]) / 2;

        light.length = cv::norm(top - bottom);
        light.width = cv::norm(p[0] - p[1]);

        light.tiltAngle = std::atan2(std::fabs(top.x - bottom.x), std::fabs(top.y - bottom.y));
        light.tiltAngle /= (CV_PI * 180);

        float ratio = light.width / light.length;
        bool ratioOK = mConfig.minLightRatio < ratio && ratio < mConfig.maxLightRatio;
        bool angleOK = light.tileAngle < mConfig.maxLightAngle;

        if(ratioOK && angleOK) {
            return light;
        } else {
            return {};
        }
    }

    std::optional<Armor> judgeArmor(const Light& light1, const Light& light2) {

        // Ratio of the length of 2 lights (short side / long side)
        float lightLenRation = light1.length < light2.length ? light1.length / light2.length : light2.length / light1.length;
        bool lightRatioOK = lightLenRation > mConfig.min2lightLenRatio;

        const auto clcCenter = [](auto&& p1, auto&& p2) { return cv::Point2f((p1.x + p2.x) / 2, (p1.y + p2.y) / 2); };
        cv::Point2f light1Center = clcCenter(light1.top, light1.bottom), light2Center = clcCenter(light2.top, light2.bottom);
        // Distance between the center of 2 lights (unit : light length)
        float avgLightLen = (light1.length + light2.length) / 2;
        float armorWidth = cv::norm(light1Center, light2Center);
        float armorRatio = armorWidth / avgLightLen;
        bool armorRatioOK = (mConfig.minSmallArmorRation < armorRatio && armorRatio < mConfig.minSmallArmorRation) ||
            (mConfig.minLargeArmorRation < armorRatio && armorRatio < mConfig.minLargeArmorRatio);

        // Angle of light center connection
        cv::Point2f diff = light1Center - light2Center;
        float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
        bool angleOK = angle < mConifg.maxArmorAngle;

        bool isArmor = lightRatioOK && armorRatioOK && angleOK;
        if(!isArmor)
            return {};

        Armor aromr;
        armor.leftLight = light1;
        armor.rightLight = light2;
        if(light1Center.x > light2Center.x) {
            std::swap(armor.leftLight, armor.rightLight);
        }
        armor.ArmorType = armorRatio > a.minLargeArmorRation ? true : false;
        return armor;
    }

    // Check if there is another light in the boundingRect formed by the 2 lights
    bool containLight(const Light& light1, const Light& light2, const std::vector<Light>& lights) {
        auto points = std::vector<cv::Point2f>{ light1.top, light1.bottom, light2.top, light2.bottom };
        auto boundingRect = cv::boundingRect(points);

        for(const auto& testLight : lights) {
            if(testLight.center == light1.center || testLight.center == light2.center)
                continue;

            if(boundingRect.contains(testLight.top) || boundingRect.contains(testLight.bottom) ||
               boundingRect.contains(testLight.center)) {
                return true;
            }
        }

        return false;
    }

    std::vector<Armors> solve(const cv::Mat& image) {
        const auto binaryImg = binary(image);
        if(mConfig.debugView) {
            debugView("binary", binaryImg, [](auto&) {});
        }

        const auto lights = findLights(image, lightPart);
        auto armors = matchLights(lights);

        if(!armors.empty()) {
            const auto imgs = numClassifier->extractNumbers(images, armors);
            numClassifier->classify(armors, imgs);
        }

        return armors;
    }

    cv::Mat binary(const cv::Mat& src) {
        cv::Mat grayImg;
        cv::cvtColor(src, grayImg, cv::COLOR_BGR2GRAY);

        cv::Mat binaryImg;
        cv::threshold(grayImg, binaryImg, mConfig.binaryThresh, 255, cv::THRESH_BINARY);

        // cv::medianBlur(result, result, 3);
        //        debugView("binary", result, [](auto) {});
        return binaryImg;
    }

    std::vector<Light> findLights(const cv::Mat& rgbImg, const cv::Mat& binary) {

        std::vector<std::vector<cv::Point2i>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<Light> lights;
        for(auto& lightContour : contours) {
            if(lightContour.size() < 5)
                continue;

            auto rRect = cv::minAreaRect(lightContour);
            auto light = isLight(rRect);
            if(light.hasValue()) {
                auto rect = rRect.boundingRect();
                if(0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rgbImg.cols && 0 <= rect.y && 0 <= rect.height &&
                   rect.y + rect.height <= rgbImg.rows) {
                    int sumR = 0, sumB = 0;
                    auto roi = rgbImg(rect);

                    for(int i = 0; i < roi.rows; i++) {
                        for(int j = 0; j < roi.cols; j++) {
                            if(cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
                                // if point is inside contour
                                sumR += roi.at<cv::Vec3b>(i, j)[0];
                                sumB += roi.at<cv::Vec3b>(i, j)[2];
                            }
                        }
                    }
                    // Sum of red pixels > sum of blue pixels ?
                    light.color = sumR > sumB ? COLOR::RED : COLOR::BLUE;
                    lights.emplace_back(light);
                }
            }
        }

        if(mConfig.debugView) {
            debugView("contour", binary, [&](cv::Mat& src) {
                for(auto& light : lights) {
                    cv::line(src, light.top, light.bottom, cv::Scalar(0, 255, 255), 1);
                    cv::putText(src, fmt::format("({:.2f}", light.ratio),
                                { static_cast<int32_t>(light.top.x), static_cast<int32_t>(light.top.y) },
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar{ 255 });
                    cv::putText(src, fmt::format("({:.2f}", light.titlAngle),
                                { static_cast<int32_t>(light.top.x), static_cast<int32_t>(light.top.y) },
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar{ 255 });
                }
            });
        }

        return lights;
    }

    std::vector<Armor> matchLights([[maybe_unused]] const cv::Mat& src, const std::vector<cv::Light>& lights) {
        auto selfColor = GlobalSettings::get().selfColor;
        std::vector<Armor> armors;
        for(auto light1 = lights.begin(); light1 != lights.end(); light1++) {
            for(auto light2 = light1 + 1; light2 != lights.end(); light2++) {
                if(light1->color == selfColor || light2->color != selfColor)
                    continue;

                if(containLight(*light1, *light2, lights)) {
                    continue;
                }

                auto armor = judgeArmor(light1, light2);
                if(armor.hasValue())
                    armors.emplace_back(armor);
            }
        }

        return armors;
    }

public:
    ArmorDetector(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](image_frame_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame, std::string_view>);
                     ACTOR_EXCEPTION_PROBE();

                     const auto frame = std::get<0>(BlackBoard::instance().get<CameraFrame, std::string_view>(key).value());

                     DetectedArmorArray res;
                     res.frame = frame;
                     res.armors = solve(frame.frame);
                     //                     logInfo("Detector works well");
                     sendAll(armor_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 } };
    }
};

HUB_REGISTER_CLASS(ArmorDetector);
