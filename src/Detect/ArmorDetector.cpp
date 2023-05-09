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
    float maxLightWidth;
    float minLightRectRatio;   // width/height
    float maxLightRectRatio;   // width/height
    float maxLightAngle;       // angle(degree)
    float min2lightLenRatio;   // light1.height / light2.height
    float minArmorRectRatio;   // width/height
    float maxArmorRectRatio;   // width/height
    float maxArmorAngle;       // angle(degree)
    float minLargeArmorRatio;  // width / height
    std::string numClassifyModelPath;
    float numProbThresh;  // number classify probability threshold
};

constexpr float fontScale = 1.5;

struct CondidateArmor final {
    bool isLargeArmor;
    uint32_t leftLightIdx;
    uint32_t rightLightIdx;
    int id;
    float ratio;
    float angle;
    float prob;
    std::vector<cv::Point2f> points;
};

template <class Inspector>
bool inspect(Inspector& f, ArmorDetectorSettings& x) {
    return f.object(x).fields(
        f.field("debugView", x.debugView).fallback(false), f.field("binaryThresh", x.binaryThresh).fallback(100),
        f.field("maxLightLen", x.maxLightLen).fallback(50.0), f.field("maxLightWidth", x.maxLightWidth).fallback(10.0),
        f.field("minLightRectRatio", x.minLightRectRatio).fallback(0.15),
        f.field("maxLightRectRatio", x.maxLightRectRatio).fallback(0.6), f.field("maxLightAngle", x.maxLightAngle).fallback(40),
        f.field("min2lightLenRatio", x.min2lightLenRatio).fallback(0.6),
        f.field("minArmorRectRatio", x.minArmorRectRatio).fallback(0.8),
        f.field("maxArmorRectRatio", x.maxArmorRectRatio).fallback(5.0), f.field("maxArmorAngle", x.maxArmorAngle).fallback(15.0),
        f.field("minLargeArmorRatio", x.minLargeArmorRatio).fallback(3.2),
        f.field("numClassifyModelPath", x.numClassifyModelPath), f.field("numProbThresh", x.numProbThresh).fallback(0.7));
}

// reference: https://github.com/chenjunnn/rm_auto_aim
class ArmorDetector final
    : public HubHelper<caf::event_based_actor, ArmorDetectorSettings, armor_detect_available_atom, image_frame_atom> {
    Identifier mKey;
    std::unique_ptr<NumberClassifier> mNumClassifierPtr;
    std::vector<Light> mDebugLights;

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

    std::vector<Armor> solve(const cv::Mat& image) {
        const auto binaryImg = binary(image);
        if(mConfig.debugView) {
            debugView("binary", binaryImg, [](auto&) {});
        }

        auto lights = findLights(image, binaryImg);
        std::sort(lights.begin(), lights.end(), [](const auto& l1, const auto& l2) { return l1.center.x < l2.center.x; });

        auto armors = matchLights(image, lights);

        return armors;
    }

    cv::Mat binary(const cv::Mat& src) {
        cv::Mat grayImg;
        cv::cvtColor(src, grayImg, cv::COLOR_BGR2GRAY);

        cv::Mat binaryImg;
        cv::threshold(grayImg, binaryImg, mConfig.binaryThresh, 255, cv::THRESH_BINARY);

        //        debugView("binary", result, [](auto) {});
        return binaryImg;
    }

    std::vector<Light> findLights(const cv::Mat& bgrImg, const cv::Mat& binary) {
        mDebugLights.clear();

        auto selfColor = GlobalSettings::get().getColor();
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
                                auto b = static_cast<int>(roi.at<cv::Vec3b>(i, j)[0]),
                                     r = static_cast<int>(roi.at<cv::Vec3b>(i, j)[2]);
                                if(b - r > 0) {
                                    ++sumB;
                                } else {
                                    ++sumR;
                                }
                            }
                        }
                    }
                    light->color = sumR > sumB ? Color::Red : Color::Blue;
                    if(light->color == selfColor || light->color == Color::Negative)
                        continue;
                    lights.emplace_back(light.value());
                }
            }
        }

        if(mConfig.debugView) {
            if(!lights.empty()) {
                debugView("Lights", bgrImg, [&](cv::Mat& src) {
                    for(auto& light : lights) {
                        cv::line(src, light.top, light.bottom, cv::Scalar(0, 255, 255), 1);
                        cv::putText(src, fmt::format("{:.2f}, {:.2f}", light.ratio, light.tiltAngle),
                                    { static_cast<int32_t>(light.top.x), static_cast<int32_t>(light.top.y) },
                                    cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar{ 0, 255, 255 });
                    }
                });
            }
        }

        if(mConfig.debugView) {
            if(!mDebugLights.empty()) {
                debugView("DebugLights", bgrImg, [&](cv::Mat& src) {
                    for(auto& light : mDebugLights) {
                        cv::line(src, light.top, light.bottom, cv::Scalar(0, 255, 255), 1);
                        cv::putText(src, fmt::format("{:.2f}, {:.2f}", light.ratio, light.tiltAngle),
                                    { static_cast<int32_t>(light.top.x), static_cast<int32_t>(light.top.y) },
                                    cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar{ 0, 255, 255 });
                    }
                });
            }
        }

        return lights;
    }

    std::vector<Armor> matchLights([[maybe_unused]] const cv::Mat& bgrImg, const std::vector<Light>& lights) {

        std::vector<CondidateArmor> condArmors;
        condArmors.reserve(5);
        for(uint32_t i = 0; i < lights.size(); ++i) {
            for(uint32_t j = i + 1; j < lights.size(); ++j) {
                // Ratio of the length of 2 lights (short side / long side)
                auto light1 = lights[i], light2 = lights[j];
                float lightLenRation =
                    light1.length < light2.length ? light1.length / light2.length : light2.length / light1.length;

                if(lightLenRation < mConfig.min2lightLenRatio)
                    continue;

                // Distance between the center of 2 lights (unit : light length)
                cv::Point2f diff = light1.center - light2.center;
                float avgLightLen = (light1.length + light2.length) / 2;
                float armorWidth = cv::norm(diff);
                float armorRatio = armorWidth / avgLightLen;
                if(armorRatio < mConfig.minArmorRectRatio || armorRatio > mConfig.maxArmorRectRatio)
                    continue;

                // Angle of light center connection
                float angle = std::fabs(std::atan(diff.y / diff.x)) / CV_PI * 180;
                if(angle > mConfig.maxArmorAngle)
                    continue;

                bool isContainLights = false;
                std::vector<cv::Point2f> points = { light1.top, light1.bottom, light2.bottom, light2.top };
                for(uint32_t k = i + 1; k < j; ++k) {
                    const auto boundRect = cv::boundingRect(points);
                    if(boundRect.contains(lights[k].top) || boundRect.contains(lights[k].bottom)) {
                        isContainLights = true;
                        break;
                    }
                }
                if(isContainLights)
                    continue;

                CondidateArmor condArmor;
                condArmor.isLargeArmor = armorRatio > mConfig.minLargeArmorRatio;
                condArmor.leftLightIdx = i;
                condArmor.rightLightIdx = j;
                condArmor.id = -1;      // Not Initialise
                condArmor.prob = -1.0;  // Not Initialise
                condArmor.angle = angle;
                condArmor.ratio = armorRatio;
                condArmor.points = points;

                condArmors.push_back(std::move(condArmor));
            }
        }

        std::vector<Armor> armors;
        std::sort(condArmors.begin(), condArmors.end(),
                  [](const auto& armor1, const auto& armor2) { return armor1.angle < armor2.angle; });
        std::vector<bool> used(condArmors.size());
        // int cnt = 0;
        for(auto& condArmor : condArmors) {
            if(used[condArmor.rightLightIdx] || used[condArmor.leftLightIdx]) {
                continue;
            }
            const auto img = NumberClassifier::extractNumbers(bgrImg, condArmor.points.data(), condArmor.isLargeArmor);
            //            if (cnt++ % 20 == 0){
            //                cv::imwrite(fmt::format("record/{}.jpg",std::time(0)), img);
            //            }

            if(mConfig.debugView) {
                debugView("n", img, [](auto& src) {});
            }
//            const auto t0 = Clock::now();

            const auto [id, prob] = mNumClassifierPtr->classify(img);

//            const auto t1 = Clock::now();
//            logInfo(fmt::format("Number classification cost {:.3f}ms ", durationCastDouble(t1 - t0) * 1000));
            condArmor.id = id;
            condArmor.prob = prob;
            logInfo(fmt::format("id is {}, prob is {}", id, prob));
            if(id == 8 || prob < mConfig.numProbThresh)  // id 8 -> negative
                continue;
            Armor armor = {};
            armor.light4Point = condArmor.points;
            armor.robotType = static_cast<RobotType>(id);
            armor.prob = prob;
            used[condArmor.leftLightIdx] = true;
            used[condArmor.rightLightIdx] = true;
            armors.push_back(armor);
        }

        if(mConfig.debugView) {
            if(!condArmors.empty()) {
                debugView("Armors", bgrImg, [&](cv::Mat& src) {
                    for(auto& armor : condArmors) {
                        for(int i = 0; i < 4; ++i) {
                            cv::line(src, armor.points[i], armor.points[(i + 1) % 4], cv::Scalar(0, 0, 255));
                        }
                        cv::putText(src, fmt::format("ratio:{:.2f}, angle:{:.2f}", armor.ratio, armor.angle),
                                    { static_cast<int32_t>(armor.points[0].x), static_cast<int32_t>(armor.points[0].y) },
                                    cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar{ 0, 255, 255 });
                        cv::putText(src, fmt::format("id:{}, prob:{:.2f}", armor.id, armor.prob),
                                    { static_cast<int32_t>(armor.points[1].x), static_cast<int32_t>(armor.points[1].y) },
                                    cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar{ 0, 255, 255 });
                    }
                });
            }
        }

        return armors;
    }

public:
    ArmorDetector(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        mNumClassifierPtr = std::make_unique<NumberClassifier>(mConfig.numClassifyModelPath);
    }

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](image_frame_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame, std::string_view>);
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
