#include "BlackBoard.hpp"
#include "ClassifiedNum.hpp"
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
    bool debugView;
    int32_t binaryThresh;
    float maxLightLen;
    float minLightRectRatio;   // width/height
    float maxLightRectRatio;   // width/height
    float maxLightAngle;       // angle(degree)
    float min2lightLenRatio;   // light1.height / light2.height
    float minArmorRectRatio;   // width/height
    float maxArmorRectRatio;   // width/height
    float maxArmorAngle;       // angle(degree)
    float minLargeArmorRatio;  // width / height
    std::string numClassifyModelPath;
    std::string numClassifyLabelPath;
    float numConfThresh;
};

struct DebugArmor final {
    std::vector<cv::Point2f> points;
    float ratio;
    float angle;
};

template <class Inspector>
bool inspect(Inspector& f, ArmorDetectorSettings& x) {
    return f.object(x).fields(
        f.field("debugView", x.debugView).fallback(false), f.field("binaryThresh", x.binaryThresh).fallback(100),
        f.field("maxLightLen", x.maxLightLen).fallback(50.0), f.field("minLightRectRatio", x.minLightRectRatio).fallback(0.15),
        f.field("maxLightRectRatio", x.maxLightRectRatio).fallback(0.6), f.field("maxLightAngle", x.maxLightAngle).fallback(40),
        f.field("min2lightLenRatio", x.min2lightLenRatio).fallback(0.6),
        f.field("minArmorRectRatio", x.minArmorRectRatio).fallback(0.8),
        f.field("maxArmorRectRatio", x.maxArmorRectRatio).fallback(5.0), f.field("maxArmorAngle", x.maxArmorAngle).fallback(15.0),
        f.field("minLargeArmorRatio", x.minLargeArmorRatio).fallback(3.2),
        f.field("numClassifyModelPath", x.numClassifyModelPath), f.field("numClassifyLabelPath", x.numClassifyLabelPath),
        f.field("numConfThresh", x.numConfThresh).fallback(0.7));
}


// reference: https://github.com/chenjunnn/rm_auto_aim
class ArmorDetector final
    : public HubHelper<caf::event_based_actor, ArmorDetectorSettings, armor_detect_available_atom, image_frame_atom> {
    Identifier mKey;
    std::unique_ptr<NumberClassifier> mNumClassifierPtr;
    std::vector<Light> mDebugLights;
    std::vector<DebugArmor> mDebugArmors;  // 4 point, ratio, angle

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

    std::optional<Light> isLight(const cv::RotatedRect& lightRect) {

        const auto clcCenter = [](auto&& p1, auto&& p2) { return cv::Point2f((p1.x + p2.x) / 2, (p1.y + p2.y) / 2); };

        Light light;
        cv::Point2f p[4];
        lightRect.points(p);
        std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
        light.top = (p[0] + p[1]) / 2;
        light.bottom = (p[2] + p[3]) / 2;
        light.center = clcCenter(light.top, light.bottom);

        light.length = cv::norm(light.top - light.bottom);
        light.width = cv::norm(p[0] - p[1]);
        bool lenOK = std::max(light.length, light.width) < mConfig.maxLightLen;

        light.tiltAngle = std::atan2(std::fabs(light.top.x - light.bottom.x), std::fabs(light.top.y - light.bottom.y));
        light.tiltAngle /= (CV_PI * 180);

        light.ratio = light.width / light.length;

        bool ratioOK = mConfig.minLightRectRatio < light.ratio && light.ratio < mConfig.maxLightRectRatio;
        bool angleOK = light.tiltAngle < mConfig.maxLightAngle;

        if(mConfig.debugView) {
            mDebugLights.push_back(light);
        }

        if(ratioOK && angleOK && lenOK) {
            return light;
        } else {
            return {};
        }
    }

    std::optional<Armor> judgeArmor(const Light& light1, const Light& light2) {

        // Ratio of the length of 2 lights (short side / long side)
        float lightLenRation = light1.length < light2.length ? light1.length / light2.length : light2.length / light1.length;
        bool lightRatioOK = lightLenRation > mConfig.min2lightLenRatio;

        // Distance between the center of 2 lights (unit : light length)
        cv::Point2f diff = light1.center - light2.center;
        float avgLightLen = (light1.length + light2.length) / 2;
        float armorWidth = cv::norm(diff);
        float armorRatio = armorWidth / avgLightLen;
        bool armorRatioOK = (mConfig.minArmorRectRatio < armorRatio && armorRatio < mConfig.maxArmorRectRatio);

        // Angle of light center connection
        float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
        bool angleOK = angle < mConfig.maxArmorAngle;

        bool isArmor = lightRatioOK && armorRatioOK && angleOK;
        if(!isArmor)
            return {};

        Armor armor;
        armor.leftLight = light1;
        armor.rightLight = light2;
        armor.center = (light1.center + light2.center) / 2;
        if(light1.center.x > light2.center.x) {
            std::swap(armor.leftLight, armor.rightLight);
        }
        if(mConfig.debugView) {
            DebugArmor darmor;
            darmor.points = { armor.leftLight.top, armor.leftLight.bottom, armor.rightLight.bottom, armor.rightLight.top };
            darmor.ratio = armorRatio;
            darmor.angle = angle;
            mDebugArmors.emplace_back(std::move(darmor));
        }
        armor.armorType = armorRatio > mConfig.minLargeArmorRatio ? ArmorType::Large : ArmorType::Small;

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

    std::vector<Armor> solve(const cv::Mat& image) {
        const auto binaryImg = binary(image);
        if(mConfig.debugView) {
            debugView("binary", binaryImg, [](auto&) {});
        }

        const auto lights = findLights(image, binaryImg);
        auto armors = matchLights(image, lights);

        if(!armors.empty()) {
            const auto imgs = mNumClassifierPtr->extractNumbers(image, armors);
            mNumClassifierPtr->classify(armors, imgs);
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

    std::vector<Light> findLights(const cv::Mat& bgrImg, const cv::Mat& binary) {
        mDebugLights.clear();

        std::vector<std::vector<cv::Point2i>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<Light> lights;
        for(auto& lightContour : contours) {
            if(lightContour.size() < 5)
                continue;

            auto rRect = cv::minAreaRect(lightContour);
            auto light = isLight(rRect);
            if(light.has_value()) {
                auto rect = rRect.boundingRect();
                if(0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= bgrImg.cols && 0 <= rect.y && 0 <= rect.height &&
                   rect.y + rect.height <= bgrImg.rows) {
                    int sumR = 0, sumB = 0;
                    auto roi = bgrImg(rect);

                    for(int i = 0; i < roi.rows; i++) {
                        for(int j = 0; j < roi.cols; j++) {
                            if(cv::pointPolygonTest(lightContour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
                                // if point is inside contour
                                sumB += roi.at<cv::Vec3b>(i, j)[0];
                                sumR += roi.at<cv::Vec3b>(i, j)[2];
                            }
                        }
                    }
                    // Sum of red pixels > sum of blue pixels ?
                    light->color = sumR > sumB ? Color::Red : Color::Blue;
                    lights.emplace_back(std::move(light.value()));
                }
            }
        }

        if(mConfig.debugView) {
            if(mDebugLights.size() > 0) {
                debugView("lights", bgrImg, [&](cv::Mat& src) {
                    for(auto& light : mDebugLights) {
                        cv::line(src, light.top, light.bottom, cv::Scalar(0, 0, 255), 1);
                        cv::putText(src, fmt::format("{:.2f}", light.ratio),
                                    { static_cast<int32_t>(light.top.x), static_cast<int32_t>(light.top.y) },
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar{ 0, 0, 255 });
                        cv::putText(src, fmt::format("{:.2f}", light.tiltAngle),
                                    { static_cast<int32_t>(light.bottom.x), static_cast<int32_t>(light.bottom.y) },
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar{ 0, 0, 255 });
                        //                        cv::putText(src, fmt::format("{:.2f}", light.length),
                        //                                    { static_cast<int32_t>(light.center.x),
                        //                                    static_cast<int32_t>(light.center.y) }, cv::FONT_HERSHEY_SIMPLEX,
                        //                                    0.3, cv::Scalar{ 0, 0, 255 });
                    }
                });
            }
        }

        return lights;
    }

    std::vector<Armor> matchLights([[maybe_unused]] const cv::Mat& bgrImg, const std::vector<Light>& lights) {
        mDebugArmors.clear();

        auto selfColor = GlobalSettings::get().selfColor;
        std::vector<Armor> armors;
        for(auto light1 = lights.begin(); light1 != lights.end(); light1++) {
            for(auto light2 = light1 + 1; light2 != lights.end(); light2++) {
                if(light1->color == selfColor || light2->color == selfColor)
                    continue;

                if(containLight(*light1, *light2, lights)) {
                    continue;
                }

                auto armor = judgeArmor(*light1, *light2);
                if(armor.has_value())
                    armors.push_back(std::move(armor.value()));
            }
        }

        if(mConfig.debugView) {
            if(mDebugArmors.size() > 0) {
                debugView("Armors", bgrImg, [&](cv::Mat& src) {
                    for(auto& armor : mDebugArmors) {
                        for(int i = 0; i < 4; ++i) {
                            cv::line(src, armor.points[i], armor.points[(i + 1) % 4], cv::Scalar(0, 255, 255));
                        }
                        cv::putText(src, fmt::format("{:.2f}", armor.ratio),
                                    { static_cast<int32_t>(armor.points[0].x), static_cast<int32_t>(armor.points[0].y) },
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar{ 0, 0, 255 });
                        cv::putText(src, fmt::format("{:.2f}", armor.angle),
                                    { static_cast<int32_t>(armor.points[1].x), static_cast<int32_t>(armor.points[1].y) },
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar{ 0, 0, 255 });
                    }
                });
            }
        }

        return armors;
    }

public:
    ArmorDetector(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        mNumClassifierPtr =
            std::make_unique<NumberClassifier>(mConfig.numClassifyModelPath, mConfig.numClassifyLabelPath, mConfig.numConfThresh);
    }

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](image_frame_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame>);
                     ACTOR_EXCEPTION_PROBE();

                     const auto t1 = Clock::now();
                     const auto data = BlackBoard::instance().get<CameraFrame, std::string_view>(key).value();
                     auto frame = std::get<0>(data);

                     DetectedArmorArray res;
                     res.frame = frame;
                     res.armors = solve(frame.frame);
                     const auto t2 = Clock::now();
                     logInfo(fmt::format("Armor Detector Cost time: {:.3f}s", durationCastDouble(t2 - t1)));
                     sendAll(armor_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 } };
    }
};

HUB_REGISTER_CLASS(ArmorDetector);
