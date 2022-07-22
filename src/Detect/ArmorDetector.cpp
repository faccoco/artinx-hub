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
    std::vector<int32_t> thresholdForBlue;  // minBlue, maxGreen, maxRed
    std::vector<int32_t> thresholdForRed;   // minRed, maxBlue,maxGreen
    float maxAreaRatio;                     // ellipseArea/contourArea
    float maxLightAngle;                    // cos(angle)
    float maxLightRectRatio;                // height/width
    float maxArmorRectRatio;                // width/ (1.8 *  height)
    float maxArmorAngle;                    // cos(angle)
    float minLightBaseAngle;                // cos(lightAngle-armorAngle)
    float maxParallelAngle;                 // sin(lightAngle1-lightAngle2)
    float minLightHeightRatio;              // lightHeight/armorHeight
};

template <class Inspector>
bool inspect(Inspector& f, ArmorDetectorSettings& x) {
    return f.object(x).fields(f.field("globalScale", x.globalScale),
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

        sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(newKey, std::move(frame)));
    }

    std::vector<PairedLight> solve(const cv::Mat& image) {
        ACTOR_EXCEPTION_PROBE();
        const cv::Mat scaled = image * mConfig.globalScale;
        // debugView("scaled",scaled,[](auto&){});
        const auto lightPart = binary(scaled);

        /*
        cv::Mat color;
        image.copyTo(color, lightPart);
        debugView("color", color, [](auto&) {});
        */

        const auto lights = findLights(image, lightPart);
        return matchLights(image, lights);
    }

    static bool isWhite(int32_t b, int32_t g, int32_t r) {
        return b + g + r > 520;
    }

    cv::Mat binary(const cv::Mat& src) {
        cv::Mat result(src.size(), CV_8U);
        if(GlobalSettings::get().selfColor == Color::Blue) {//modify this to change color
            const auto minB = mConfig.thresholdForBlue[0];
            const auto maxG = mConfig.thresholdForBlue[1];
            const auto maxR = mConfig.thresholdForBlue[2];

            for(int32_t i = 0; i < src.rows; ++i)
                for(int32_t j = 0; j < src.cols; ++j) {
                    const auto& col = src.at<cv::Vec3b>(i, j);
                    const int32_t b = col[0], g = col[1], r = col[2];
                    result.at<uchar>(i, j) = (r > minB && g < maxG && r < maxR && r * 3 > g + b) ? 255 : 0;
                }
        } else {
            const auto minR = mConfig.thresholdForRed[0];
            const auto maxB = mConfig.thresholdForRed[1];
            const auto maxG = mConfig.thresholdForRed[2];

            for(int32_t i = 0; i < src.rows; ++i)
                for(int32_t j = 0; j < src.cols; ++j) {
                    const auto& col = src.at<cv::Vec3b>(i, j);
                    const int32_t b = col[0], g = col[1], r = col[2];
                    result.at<uchar>(i, j) = ( b > minR && b < maxB && g < maxG && b * 3 > r + g) ? 255 : 0;
                }
        }
        cv::Mat blurred;
        cv::medianBlur(result, blurred, 5);
        //debugView("binary", blurred, [](auto&) {});
        return blurred;
    }

    void fixContour(const cv::Mat& color, const cv::Mat& binary, std::vector<cv::Point2i>& contour) {
        const auto bound = cv::boundingRect(contour);

        const auto subColor = color(bound);
        const auto subBinary = binary(bound);

        uint8_t maxB = 0;
        for(int32_t y = 0; y < subColor.rows; ++y) {
            for(int32_t x = 0; x < subColor.cols; ++x) {
                if(subBinary.at<cv::uint8_t>(y, x)) {
                    const auto& col = subColor.at<cv::Vec3b>(y, x);
                    maxB = std::max(maxB, col[0]);
                }
            }
        }

        const auto threshold = static_cast<uint8_t>(maxB * 0.5);
        std::vector<cv::Point2i> points;
        points.reserve(subColor.rows * 2);

        for(int32_t y = 0; y < subColor.rows; ++y) {
            bool last = false;

            for(int32_t x = 0; x < subColor.cols; ++x) {
                if(subBinary.at<cv::uint8_t>(y, x)) {
                    const auto& col = subColor.at<cv::Vec3b>(y, x);
                    const auto cur = col[0] >= threshold;
                    if(cur != last) {
                        last = cur;
                        points.emplace_back(x + bound.x, y + bound.y);
                    }
                }
            }
        }

        cv::convexHull(points, contour);
    }

    static cv::RotatedRect correctRectAngle(cv::RotatedRect lightRect) {
        if(std::fmax(lightRect.size.width, lightRect.size.height) < 5.0f) {
            if(std::fabs(std::sin(glm::radians(lightRect.angle))) < glm::root_two<float>() * 0.5f) {
                std::swap(lightRect.size.width, lightRect.size.height);
                lightRect.angle += 90.0f;
            }
        } else if(std::fmax(lightRect.size.width, lightRect.size.height) /
                      std::fmin(lightRect.size.width, lightRect.size.height) >
                  1.2f) {
            if((lightRect.size.width > 1.5f * lightRect.size.height) ||
               (lightRect.size.width > 1.2f * lightRect.size.height &&
                std::fabs(std::sin(glm::radians(lightRect.angle))) < glm::root_two<float>() * 0.5f)) {
                std::swap(lightRect.size.width, lightRect.size.height);
                lightRect.angle += 90.0f;
            }
        }
        return lightRect;
    }

    std::vector<cv::RotatedRect> findLights(const cv::Mat& color, const cv::Mat& binary) {
        ACTOR_EXCEPTION_PROBE();
        const cv::Rect full = { 0, 0, color.cols, color.rows };

        // TODO: down sampling
        std::vector<std::vector<cv::Point2i>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<cv::RotatedRect> lights;
        for(auto& lightContour : contours) {
            // if(cv::contourArea(lightContour) < 300.0)
            //     fixContour(color, binary, lightContour);
            const auto rawRect = cv::minAreaRect(lightContour);

            if(rawRect.size.width < 0.5f || rawRect.size.height < 0.5f)
                continue;

            auto lightRect = correctRectAngle(rawRect);

            if(lightContour.size() >= 6) {
                const auto rect = correctRectAngle(cv::fitEllipse(lightContour));
                if(rect.size.area() > 4.0f &&
                   std::fabs(std::sin(glm::radians(lightRect.angle))) > std::fabs(std::sin(glm::radians(rect.angle))))
                    lightRect = rect;
            }

            if(std::fmax(lightRect.size.width, lightRect.size.height) < 10.0f)
                continue;
            if(std::fmin(lightRect.size.width, lightRect.size.height) > 50.0f)
                continue;

            const auto rect = lightRect.boundingRect();
            if((rect & full) != rect)
                continue;

            /*
            if(static_cast<double>(lightRect.size.width) * static_cast<double>(lightRect.size.height) *
                   glm::quarter_pi<double>() >
               mConfig.maxAreaRatio * cv::contourArea(lightContour))
                continue;
            */

            if(lightRect.size.height > 2.0f * lightRect.size.width &&
               std::fabs(std::cos(glm::radians(lightRect.angle))) < mConfig.maxLightAngle) {
                continue;
            }
            if(lightRect.size.height > mConfig.maxLightRectRatio * lightRect.size.width && lightRect.size.width > 3.0f)
                continue;

            lights.emplace_back(lightRect);
        }

        /*
        debugView("contour", color, [&](cv::Mat &src) {
            for (auto &light: lights)
                cv::ellipse(src, light, cv::Scalar{255, 0, 255}, 1);
            cv::drawContours(src, contours, -1, cv::Scalar{0, 255, 0}, 1);
        });
        */

        std::sort(lights.begin(), lights.end(), [](const auto& lhs, const auto& rhs) { return lhs.center.x < rhs.center.x; });
        return lights;
    }

    cv::Rect2f boundingRect(const PairedLight& armor) {
        const auto b1 = armor.r1.boundingRect2f();
        const auto b2 = armor.r2.boundingRect2f();
        return b1 | b2;
    }

    std::vector<PairedLight> removeReflected(std::vector<PairedLight> armors) {
        std::sort(armors.begin(), armors.end(), [](const PairedLight& lhs, const PairedLight& rhs) {
            return lhs.r1.center.y + lhs.r2.center.y < rhs.r1.center.y + rhs.r2.center.y;
        });

        std::vector<PairedLight> res;
        res.reserve(armors.size());
        std::vector<cv::Rect2f> exceptBounds;
        exceptBounds.reserve(armors.size());
        for(auto& armor : armors) {
            auto bound = boundingRect(armor);

            bool flag = true;
            for(auto& exceptBound : exceptBounds) {
                if((bound & exceptBound) == bound) {
                    flag = false;
                    break;
                }
            }
            if(!flag)
                continue;

            bound.x -= bound.width * 0.25f;
            bound.width *= 1.5f;
            bound.y -= bound.height * 0.15f;
            bound.height *= 5.0f;
            exceptBounds.push_back(bound);
            res.push_back(armor);
        }
        return res;
    }

    std::vector<PairedLight> matchLights([[maybe_unused]] const cv::Mat& src, const std::vector<cv::RotatedRect>& lights) {
        ACTOR_EXCEPTION_PROBE();
        std::vector<std::tuple<uint32_t, uint32_t, float>> pairs;
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
                    rect.angle += 90.0f;
                }
                if(rect.size.height < 3.0f)
                    continue;

                /*
                {
                    uninstallFPEProbe();
                    bool flag = true;
                    std::vector<cv::Point2f> intersect;
                    for(uint32_t k = i + 1; k < j; ++k) {
                        const auto& mid = lights[k];
                        if(cv::rotatedRectangleIntersection(mid, rect, intersect) !=
                           cv::RectanglesIntersectTypes::INTERSECT_NONE) {
                            flag = false;
                            continue;
                        }
                    }
                    installFPEProbe();
                    if(!flag)
                        continue;
                }
                */

                constexpr auto largeRatio = widthOfLargeArmor / heightOfArmorLightBar;
                constexpr auto smallRatio = widthOfSmallArmor / heightOfArmorLightBar;

                const auto ratio = rect.size.aspectRatio();
                auto diff = static_cast<float>(std::fabs(ratio - smallRatio) / smallRatio);
                const auto diffLarge = static_cast<float>(std::fabs(ratio - largeRatio) / largeRatio);
                bool largeArmor = false;
                if(diffLarge < diff) {
                    diff = diffLarge;
                    largeArmor = true;
                }

                if(diff > mConfig.maxArmorRectRatio)
                    continue;
                const auto rectAngle = std::atan2(lhs.center.y - rhs.center.y, lhs.center.x - rhs.center.x);

                if(std::fabs(std::cos(rectAngle)) < mConfig.maxArmorAngle)
                    continue;
                const auto area1 = lhs.size.area();
                const auto area2 = rhs.size.area();
                auto par = std::fmin(area1, area2) / std::fmax(area1, area2);

                if(std::fmax(lhs.size.aspectRatio(), rhs.size.aspectRatio()) < 0.5 && par < 0.1f)
                    continue;

                if(area1 + area2 > 0.5f * rect.size.area())
                    continue;

                if(std::fmax(lhs.size.aspectRatio(), rhs.size.aspectRatio()) < 0.5) {
                    if(std::fabs(std::cos(rectAngle - glm::radians(lhs.angle))) < mConfig.minLightBaseAngle)
                        continue;
                    if(std::fabs(std::cos(rectAngle - glm::radians(rhs.angle))) < mConfig.minLightBaseAngle)
                        continue;

                    par = std::fabs(std::sin(glm::radians(lhs.angle) - glm::radians(rhs.angle)));

                    if(par > mConfig.maxParallelAngle)
                        continue;
                    if(par > 0.2f && std::tan(glm::radians(lhs.angle)) * std::tan(glm::radians(rhs.angle)) < 0.0f)
                        continue;
                }

                if(std::fmax(lhs.size.height, rhs.size.height) > 1.2f * rect.size.height)
                    continue;

                if(std::fmin(lhs.size.height, rhs.size.height) < mConfig.minLightHeightRatio * rect.size.height)
                    continue;

                pairs.emplace_back(i, j, diff + par + (largeArmor ? 1e3f : 0.0f));
            }

        /*
        debugView("potential", src, [&](cv::Mat& frame) {
            uint32_t idx = 0;
            for(auto& light : lights) {
                cv::ellipse(frame, light, cv::Scalar{ 255, 255, 0 }, 1);
                cv::Point2f offset{ -std::sin(glm::radians(light.angle)) * light.size.height,
                                    std::cos(glm::radians(light.angle)) * light.size.height };
                const auto p0 = light.center + offset;
                const auto p1 = light.center - offset;
                cv::line(frame, p0, p1, cv::Scalar{ 255, 0, 255 });
                cv::putText(frame, std::to_string(idx++),
                            { static_cast<int32_t>(light.center.x), static_cast<int32_t>(light.center.y) },
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{ 0, 0, 255 });
            }
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
                  [](const auto& lhs, const auto& rhs) { return std::get<float>(lhs) < std::get<float>(rhs); });

        std::vector<PairedLight> res;
        std::vector<bool> used(lights.size());
        for(auto& [i, j, s] : pairs) {
            if(used[i] || used[j])
                continue;
            used[i] = used[j] = true;
            res.push_back(PairedLight{ lights[i], lights[j] });
        }

        return removeReflected(std::move(res));
    }

public:
    ArmorDetector(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}

    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                },
                 [&](car_detect_available_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(car_detect_available_atom, TypedIdentifier<DetectedCarArray>);
                     ACTOR_EXCEPTION_PROBE();

                     const auto [frame, cars] = BlackBoard::instance().get<DetectedCarArray>(key).value();

                     DetectedArmorArray res;
                     res.frame = frame;

                     for(auto& roi : cars) {
                         auto armors = solve(frame.frame(roi));
                         res.armors.push_back({ roi, 0, std::move(armors) });  // TODO: id
                     }

                     sendAll(armor_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 } };
    }
};

HUB_REGISTER_CLASS(ArmorDetector);
