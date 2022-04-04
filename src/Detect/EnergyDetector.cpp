#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "EnergyDetect.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <fmt/format.h>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

struct EnergyDetectorSettings final {
    int smallPredictMode;
    int bigPredictMode;
    float armorMinArea;
    float armorMaxArea;
    float armorMinWHRatio;
    float armorMaxWHRatio;
    float armorMinAreaRatio;
    float stripMinArea;
    float stripMaxArea;
    float stripMinWHRatio;
    float stripMaxWHRatio;
    float stripMaxAreaRatio;
    float noiseArea;
    float predictAngle;
    float radius;

    cv::Point2f offset;
};

template <class Inspector>
bool inspect(Inspector& f, EnergyDetectorSettings& x) {
    return f.object(x).fields(f.field("smallPredictMode", x.smallPredictMode), f.field("bigPredictMode", x.bigPredictMode),
                              f.field("armorMinArea", x.armorMinArea), f.field("armorMaxArea", x.armorMaxArea),
                              f.field("armorMinWHRatio", x.armorMinWHRatio), f.field("armorMaxWHRatio", x.armorMaxWHRatio),
                              f.field("armorMinAreaRatio", x.armorMinAreaRatio), f.field("stripMinArea", x.stripMinArea),
                              f.field("stripMaxArea", x.stripMaxArea), f.field("stripMaxWHRatio", x.stripMaxWHRatio),
                              f.field("stripMaxAreaRatio", x.stripMaxAreaRatio), f.field("noiseArea", x.noiseArea),
                              f.field("predictAngle", x.predictAngle), f.field("radius", x.radius),
                              f.field("offsetX", x.offset.x), f.field("offsetY", x.offset.y));
}

class EnergyDetector final
    : public HubHelper<caf::event_based_actor, EnergyDetectorSettings, energy_detect_available_atom, image_frame_atom> {

    Identifier mKey;

    bool mEnabled = false;
    int mRotateMode = 0;

    void reset() {}

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

        BlackBoard::instance().updateSync(newKey, std::move(frame));
        sendAll(image_frame_atom_v, newKey);
    }

    static void setBinary(const cv::Mat& src, cv::Mat& binary) {
        std::vector<cv::Mat> imgChannels;
        cv::split(src, imgChannels);

        /*
        constexpr auto threshold = 175;
        if(GlobalSettings::get().selfColor == Color::Red) {
            const auto energyRed = imgChannels[2] - imgChannels[0];
            cv::threshold(energyRed, binary, threshold, 255, cv::THRESH_BINARY);
        } else {
            const auto energyBlue = imgChannels[0] - imgChannels[2];
            cv::threshold(energyBlue, binary, threshold, 255, cv::THRESH_BINARY);
        }*/

        // only for test
        constexpr auto threshold = 200;
        binary = imgChannels[0] > threshold & imgChannels[1] > threshold & imgChannels[2] > threshold;
    }

    bool stripJudge(const std::vector<cv::Point>& contour, const cv::RotatedRect& rotatedRect) const {
        cv::Point2f rectPoints[4];
        rotatedRect.points(rectPoints);
        const double height = std::min(rotatedRect.size.height, rotatedRect.size.width);
        const double width = std::max(rotatedRect.size.height, rotatedRect.size.width);
        const double area = contourArea(contour);

        if(height * width > mConfig.stripMinArea && height * width < mConfig.stripMaxArea &&
           width / height < mConfig.stripMaxWHRatio && width / height > mConfig.stripMinWHRatio &&
           area / rotatedRect.size.area() < mConfig.stripMaxAreaRatio)
            return true;
        return false;
    }

    bool detectArmor(const cv::Mat& src, cv::RotatedRect& res) {
        auto binary = src.clone();
        setBinary(src, binary);

        auto element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6));
        dilate(binary, binary, element);
        // element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6));
        // erode(binary, binary, element);

        // debugView("binary", binary, [](cv::Mat&) {});

        std::vector<std::vector<cv::Point> > armorContours;
        std::vector<cv::Vec4i> armorHierarchy;
        findContours(binary, armorContours, armorHierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        const auto armorContoursSize = armorContours.size();
        if(armorContoursSize == 0) {
            logInfo("Energy detect failed");
            return false;
        }

        // debugView("contours", src, [&](cv::Mat& img) { cv::drawContours(img, armorContours, -1, cv::Scalar{ 255, 0, 0 }); });

        std::vector<uint32_t> conIndices;
        for(uint32_t i = 0; i < armorContoursSize; ++i) {
            if(contourArea(armorContours[i]) > mConfig.noiseArea) {
                if(stripJudge(armorContours[i], minAreaRect(armorContours[i]))) {
                    conIndices.push_back(i);
                }
            }
        }

        if(conIndices.empty()) {
            logInfo("Strip detect failed: no strip");
            return false;
        }

        /*
        debugView("filtered_contours", src, [&](cv::Mat& img) {
            for(auto idx : conIndices)
                cv::drawContours(img, armorContours, static_cast<int>(idx), cv::Scalar{ 255, 0, 0 });
        });*/

        uint32_t index = std::numeric_limits<uint32_t>::max();
        double minScore = 0.03;

        for(const auto conIndex : conIndices) {
            // const auto finalLength = arcLength(armorContours[conIndex], true);
            // const auto finalArea = contourArea(armorContours[conIndex]);

            const auto ratio = contourArea(armorContours[conIndex]) / cv::minAreaRect(armorContours[conIndex]).size.area();

            if(const auto score = std::fabs(ratio - 0.42); score < minScore) {
                minScore = score;
                index = conIndex;
            }
        }
        if(index == std::numeric_limits<uint32_t>::max()) {
            logInfo("Strip detect failed: no strip contour \n");
            return false;
        }
        /*
        debugView("selected_contour", src, [&](cv::Mat& img) {
            cv::drawContours(img, armorContours, static_cast<int>(index), cv::Scalar{ 255, 0, 0 });
        });*/

        const auto finalRect = boundingRect(armorContours[index]);
        const auto finalROI = binary(finalRect);
        const auto moments = cv::moments(finalROI, true);
        const auto centerX = moments.m10 / moments.m00;
        const auto centerY = moments.m01 / moments.m00;

        const auto a = moments.m20 / moments.m00 - centerX * centerX;
        const auto b = moments.m11 / moments.m00 - centerX * centerY;
        const auto c = moments.m02 / moments.m00 - centerY * centerY;

        auto rect = cv::minAreaRect(armorContours[index]);
        if(rect.size.width < rect.size.height) {
            rect.angle += 90.0;
            std::swap(rect.size.width, rect.size.height);
        }
        auto theta = 0.5 * std::atan2(2 * b, a - c);
        // auto theta = glm::radians(rect.angle);
        const auto dx = (rect.center.x - finalRect.x) - centerX, dy = (rect.center.y - finalRect.y) - centerY;
        if(dx * std::cos(theta) + dy * std::sin(theta) > 0.0)
            theta += glm::pi<double>();
        const auto cos = std::cos(theta), sin = std::sin(theta);
        const auto offset = rect.size.width * 0.27;
        debugView("finalROI", finalROI, [&](cv::Mat& img) {
            cv::circle(img, { static_cast<int>(centerX), static_cast<int>(centerY) }, 5.0, cv::Scalar{ 0 });
            cv::line(img, { static_cast<int>(centerX), static_cast<int>(centerY) },
                     { static_cast<int>(centerX + offset * cos), static_cast<int>(centerY + offset * sin) }, cv::Scalar{ 255 });
        });

        const cv::Point2f center = { static_cast<float>(finalRect.x + centerX + offset * cos),
                                     static_cast<float>(finalRect.y + centerY + offset * sin) };
        res = cv::RotatedRect{ center, { rect.size.width * 0.2f, rect.size.height * 0.8f }, rect.angle };

        if(res.size.width >= res.size.height) {
            std::swap(res.size.width, res.size.height);
            res.angle += 90.0f;
        }

        debugView("detected", src, [&](cv::Mat& img) { drawRotatedRect(img, res, cv::Scalar{ 255, 0, 0 }, 4); });

        return true;
    }

    const std::vector<cv::Point3d> mObjectPointsLarge = {
        { -widthOfLargeArmor / 2, +heightOfArmorLightBar / 2, 0.0 },
        { -widthOfLargeArmor / 2, -heightOfArmorLightBar / 2, 0.0 },
        { +widthOfLargeArmor / 2, -heightOfArmorLightBar / 2, 0.0 },
        { +widthOfLargeArmor / 2, +heightOfArmorLightBar / 2, 0.0 },
    };
    std::vector<cv::Point2f> mImagePoint{ 4 };

    Point<UnitType::Distance, FrameOfReference::Camera> solve(const cv::Mat& cameraMatrix, const cv::RotatedRect& armor) {
        armor.points(mImagePoint.data());

        const cv::Mat_<double> distCoeff;
        cv::Mat rvec, tvec;

        [[maybe_unused]] const auto res =
            cv::solvePnP(mObjectPointsLarge, mImagePoint, cameraMatrix, distCoeff, rvec, tvec, false, cv::SOLVEPNP_IPPE);
        glm::dvec3 p0 = { tvec.at<double>(0, 0), -tvec.at<double>(1, 0), -tvec.at<double>(2, 0) };

        if(p0.z > 0.0)
            p0 = -p0;

        return Point<UnitType::Distance, FrameOfReference::Camera>{ p0 };
    }

public:
    EnergyDetector(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(EnergyDetector).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    reset();
                    // only for test
                    mEnabled = true;
                    mRotateMode = 0;
                },
                 [&](energy_detector_control_atom, bool enable, int mode) {
                     if(mEnabled != enable || mRotateMode != mode)
                         reset();
                     mEnabled = enable;
                     mRotateMode = mode;
                 },
                 [&](image_frame_atom, Identifier key) {
                     if(!mEnabled)
                         return;
                     auto data = BlackBoard::instance().get<CameraFrame>(key).value();

                     cv::RotatedRect armor;
                     if(!detectArmor(data.frame, armor))
                         return;

                     const auto& cameraInfo = data.info;
                     const cv::Mat cameraMatrix =
                         (cv::Mat_<double>(3, 3) << cameraInfo.width / 2 / tan(glm::radians(cameraInfo.fov) / 2), 0,
                          cameraInfo.width / 2, 0, cameraInfo.height / 2 / tan(glm::radians(cameraInfo.fov) / 2),
                          cameraInfo.height / 2, 0, 0, 1);

                     const auto point = solve(cameraMatrix, armor);

                     const auto& transform = std::get<0>(cameraInfo.transform);

                     DetectedEnergyInfo res;
                     res.lastUpdate = data.lastUpdate;
                     res.point = transform(point);

                     const auto raw = res.point.raw();
                     std::cout << raw.x << " " << raw.y << " " << raw.z << std::endl;

                     BlackBoard::instance().updateSync(mKey, res);
                     sendAll(energy_detect_available_atom_v, mKey);
                 } };
    }
};

HUB_REGISTER_CLASS(EnergyDetector);
