#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedCar.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <fmt/format.h>
#include <magic_enum.hpp>
#include <string>
#include <utility>

struct ArmorDetectorSettings final {
    bool isRed;  // TODO: auto detect
    double globalScale;
    std::vector<int32_t> thresholdForBlue;  // minBlue, maxGreen, maxRed
    std::vector<int32_t> thresholdForRed;   // minRed, maxBlue,maxGreen
    double maxAreaRatio;                    // ellipseArea/contourArea
    double maxLightAngle;                   // cos(angle)
    double maxLightRectRatio;               // height/width
    double maxArmorRectRatio;               // width/ (1.8 *  height)
    double maxArmorAngle;                   // cos(angle)
    double minLightBaseAngle;               // cos(lightAngle-armorAngle)
    double maxParallelAngle;                // sin(lightAngle1-lightAngle2)
    double minLightHeightRatio;             // lightHeight/armorHeight
};

template <class Inspector>
bool inspect(Inspector& f, ArmorDetectorSettings& x) {
    return f.object(x).fields(f.field("isRed", x.isRed), f.field("globalScale", x.globalScale),
                              f.field("thresholdForBlue", x.thresholdForBlue).invariant([](auto& c) { return c.size() == 3; }),
                              f.field("thresholdForRed", x.thresholdForRed).invariant([](auto& c) { return c.size() == 3; }),
                              f.field("maxAreaRatio", x.maxAreaRatio), f.field("maxLightAngle", x.maxLightAngle),
                              f.field("maxLightRectRatio", x.maxLightRectRatio),
                              f.field("maxArmorRectRatio", x.maxArmorRectRatio), f.field("maxArmorAngle", x.maxArmorAngle),
                              f.field("minLightBaseAngle", x.minLightBaseAngle), f.field("maxParallelAngle", x.maxParallelAngle),
                              f.field("minLightHeightRatio", x.minLightHeightRatio));
}

class ArmorDetector final
    : public HubHelper<caf::event_based_actor, ArmorDetectorSettings, armor_detect_available_atom, image_frame_atom> {
    Identifier mKey;
    size_t mFrameCnt = 0;

    // TODO: light pairs affinity

    void debugView(const std::string_view& name, const cv::Mat& src, const std::function<void(cv::Mat&)>& func) {
#ifndef ARTINXHUB_DEBUG
        return;
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

        BlackBoard::instance().updateSync(newKey, std::move(frame));
        sendAll(image_frame_atom_v, newKey);
    }

    std::vector<PairedLight> solve(const cv::Mat& image) {
        const cv::Mat scaled = image * mConfig.globalScale;
        const auto lightPart = binary(scaled);

        /*
        cv::Mat color;
        image.copyTo(color, lightPart);
        debugView("color", color, [](auto&) {});
        */

        const auto lights = findLights(image, lightPart);
        return matchLights(image, lights);
    }

    cv::Mat binary(const cv::Mat& src) {
        cv::Mat result(src.size(), CV_8U);
        if(mConfig.isRed) {
            const auto minB = mConfig.thresholdForBlue[0];
            const auto maxG = mConfig.thresholdForBlue[1];
            const auto maxR = mConfig.thresholdForBlue[2];

            for(int32_t i = 0; i < src.rows; ++i)
                for(int32_t j = 0; j < src.cols; ++j) {
                    const auto& col = src.at<cv::Vec3b>(i, j);
                    const int32_t b = col[0], g = col[1], r = col[2];
                    result.at<uchar>(i, j) = (b > minB && g < maxG && r < maxR && b * 3 > g + r) ? 255 : 0;
                }
        } else {
            const auto minR = mConfig.thresholdForRed[0];
            const auto maxB = mConfig.thresholdForRed[1];
            const auto maxG = mConfig.thresholdForRed[2];

            for(int32_t i = 0; i < src.rows; ++i)
                for(int32_t j = 0; j < src.cols; ++j) {
                    const auto& col = src.at<cv::Vec3b>(i, j);
                    const int32_t b = col[0], g = col[1], r = col[2];
                    result.at<uchar>(i, j) = (r > minR && b < maxB && g < maxG && r * 3 > b + g) ? 255 : 0;
                }
        }
        return result;
    }

    std::vector<cv::RotatedRect> findLights(const cv::Mat& color, const cv::Mat& binary) {
        const cv::Rect full = { 0, 0, color.cols, color.rows };

        // TODO: downsampling
        std::vector<std::vector<cv::Point2i>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<cv::RotatedRect> lights;
        for(const auto& lightContour : contours) {
            if(lightContour.size() < 6)
                continue;
            auto lightRect = cv::fitEllipse(lightContour);

            const auto rect = lightRect.boundingRect();
            if((rect & full) != rect)
                continue;

            if(static_cast<double>(lightRect.size.width) * static_cast<double>(lightRect.size.height) *
                   glm::quarter_pi<double>() >
               mConfig.maxAreaRatio * cv::contourArea(lightContour))
                continue;
            if(lightRect.size.width > lightRect.size.height) {
                std::swap(lightRect.size.width, lightRect.size.height);
                lightRect.angle += 90.0;
            }

            if(lightRect.size.height > 2.0 * lightRect.size.width && lightRect.size.width > 2.0 &&
               std::fabs(std::cos(glm::radians(lightRect.angle))) < mConfig.maxLightAngle) {
                continue;
            }
            if(lightRect.size.height > mConfig.maxLightRectRatio * lightRect.size.width && lightRect.size.width > 3.0)
                continue;

            lights.emplace_back(lightRect);
        }

        /*
        debugView("contour", color, [&](cv::Mat& src) {
            cv::drawContours(src, contours, -1, cv::Scalar{ 0, 255, 0 }, 1);
            for(auto& light : lights)
                cv::ellipse(src, light, cv::Scalar{ 255, 255, 0 }, 2);
        });*/

        std::sort(lights.begin(), lights.end(), [](const auto& lhs, const auto& rhs) { return lhs.center.x < rhs.center.x; });
        //        CAF_LOG_INFO(lights.size());
        return lights;
    }

    std::vector<PairedLight> matchLights(const cv::Mat& src, const std::vector<cv::RotatedRect>& lights) {
        std::vector<std::tuple<uint32_t, uint32_t, double>> pairs;
        for(uint32_t i = 0; i < lights.size(); ++i)
            for(uint32_t j = i + 1; j < lights.size(); ++j) {
                const auto& lhs = lights[i];
                const auto& rhs = lights[j];

                std::vector<cv::Point2f> pts(8);
                lhs.points(pts.data());
                rhs.points(pts.data() + 4);

                auto rect = cv::minAreaRect(pts);
                if(rect.size.width < rect.size.height) {
                    std::swap(rect.size.width, rect.size.height);
                    rect.angle += 90.0;
                }
                if(rect.size.height < 10.0)
                    continue;

                constexpr auto maxScale =
                    std::max(widthOfLargeArmor / heightOfLargeArmor, widthOfSmallArmor / heightOfSmallArmor);

                if(rect.size.width > mConfig.maxArmorRectRatio * maxScale * rect.size.height)
                    continue;

                if(std::fabs(std::cos(glm::radians(rect.angle))) < mConfig.maxArmorAngle)
                    continue;

                if(std::fabs(std::cos(glm::radians(rect.angle) - glm::radians(lhs.angle))) < mConfig.minLightBaseAngle)
                    continue;

                if(std::fabs(std::cos(glm::radians(rect.angle) - glm::radians(rhs.angle))) < mConfig.minLightBaseAngle)
                    continue;

                const auto par = std::fabs(std::sin(glm::radians(lhs.angle) - glm::radians(rhs.angle)));

                if(par > mConfig.maxParallelAngle)
                    continue;

                if(std::min(lhs.size.height, rhs.size.height) < mConfig.minLightHeightRatio * rect.size.height)
                    continue;

                pairs.push_back({ i, j, par });
            }

        /*
        debugView("potential", src, [&](cv::Mat& frame) {
            for(auto& light : lights)
                cv::ellipse(frame, light, cv::Scalar{ 255, 255, 0 }, 1);
            for(auto& [i, j, s] : pairs) {
                const auto& lhs = lights[i];
                const auto& rhs = lights[j];

                std::vector<cv::Point2f> pts(8);
                lhs.points(pts.data());
                rhs.points(pts.data() + 4);

                auto rect = cv::minAreaRect(pts);
                if(rect.size.width < rect.size.height) {
                    std::swap(rect.size.width, rect.size.height);
                    rect.angle += 90.0;
                }

                drawRotatedRect(frame, rect, cv::Scalar{ 0, 0, 255 });
                cv::putText(frame, fmt::format("{:.3f}", rect.size.width / rect.size.height),
                            { static_cast<int32_t>(rect.center.x), static_cast<int32_t>(rect.center.y) },
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{ 255 });
            }
        });
        */

        std::sort(pairs.begin(), pairs.end(),
                  [](const auto& lhs, const auto& rhs) { return std::get<double>(lhs) < std::get<double>(rhs); });

        std::vector<PairedLight> res;
        std::vector<bool> used(lights.size());
        for(auto& [i, j, s] : pairs) {
            if(used[i] || used[j])
                continue;
            used[i] = used[j] = true;
            res.push_back(PairedLight{ lights[i], lights[j] });
        }
        return res;
    }

public:
    ArmorDetector(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(ArmorDetector).hash_code() } {}

    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](car_detect_available_atom, Identifier key) {
                     const auto data = BlackBoard::instance().get<DetectedCarArray>(key).value();

                     DetectedArmorArray res;
                     res.frame = data.frame;

                     for(auto& roi : data.cars) {
                         auto armors = solve(data.frame.frame(roi));
                         res.armors.push_back({ roi, 0, std::move(armors) });  // TODO: id
                     }

                     //                     CAF_LOG_INFO(fmt::format("ARMORS: {}", res.armors[0].armors.size()));

                     BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(armor_detect_available_atom_v, mKey);
                 } };
    }
};

HUB_REGISTER_CLASS(ArmorDetector);
