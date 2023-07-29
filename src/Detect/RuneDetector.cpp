#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedEnergyFan.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <utility>

#include "SuppressWarningEnd.hpp"

struct RuneDetecorSettings final {

    bool debugView;

    // bound for hsv
    std::vector<int> upperRed;
    std::vector<int> lowerRed;
    std::vector<int> upperBlue;
    std::vector<int> lowerBlue;

    int roiBinThresh;
    int dilateKernel;
    double minConvexHullThresh;  // for hull area ratio filter
    double maxConvexHullThresh;
    int minContourArea;          // for contour area filter
    double rRoiSizeScale;

    double rPosScale;
};

template <class Inspector>
bool inspect(Inspector& f, RuneDetecorSettings& x) {
    return f.object(x).fields(f.field("debugView", x.debugView).fallback(false), f.field("upperRed", x.upperRed),
                              f.field("lowerRed", x.lowerRed), f.field("upperBlue", x.upperBlue),
                              f.field("lowerBlue", x.lowerBlue), f.field("roiBinThresh", x.roiBinThresh),
                              f.field("dilateKernel", x.dilateKernel), f.field("minConvexHullThresh", x.minConvexHullThresh),
                              f.field("maxConvexHullThresh", x.maxConvexHullThresh), f.field("minContourArea", x.minContourArea),
                              f.field("rRoiSizeScale", x.rRoiSizeScale).fallback(1.0), f.field("rPosScale", x.rPosScale).fallback(6.5));
}

static double euclideanDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
    cv::Point diff = p1 - p2;
    return cv::sqrt(diff.x * diff.x + diff.y * diff.y);
}

static double crossProduct2D(const cv::Point2d& v1, const cv::Point2d& v2) {
    return v1.x * v2.y - v1.y * v2.x;
}

class RuneDetector final
    : public HubHelper<caf::event_based_actor, RuneDetecorSettings, energy_detect_available_atom, image_frame_atom> {

    Identifier mKey;
    bool mEnable = true;

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

    static double distance(const cv::Point2f& center1, const cv::Point2f center2) {
        return cv::norm(center2 - center1);
    }

    cv::Mat binarize(const cv::Mat& src, const cv::Scalar& lowerBound, const cv::Scalar& upperBound) {
        cv::Mat hsvImg, bin;
        cv::cvtColor(src, hsvImg, cv::COLOR_BGR2HSV);
        cv::inRange(hsvImg, lowerBound, upperBound, bin);
        return bin;
    }

    cv::Mat binarize(const cv::Mat& src, double binThresh) {
        cv::Mat bin;
        cv::cvtColor(src, bin, cv::COLOR_BGR2GRAY);
        cv::threshold(bin, bin, binThresh, 255, cv::THRESH_BINARY);
        return bin;
    }

    std::optional<cv::Rect2f> getRuneROI(const cv::Mat& src) {
        cv::Mat bin = binarize(src, cv::Scalar(mConfig.lowerBlue[0], mConfig.lowerBlue[1], mConfig.lowerBlue[2]),
                               cv::Scalar(mConfig.upperBlue[0], mConfig.upperBlue[1], mConfig.upperBlue[2]));

        // cv::Mat bin = binarize(src, mConfig.binThresh);

        if(mConfig.debugView) {
            debugView("bin", bin, [](auto&) {});
        }

        auto openKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        auto closeKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(mConfig.dilateKernel, mConfig.dilateKernel));
        cv::morphologyEx(bin, bin, cv::MORPH_OPEN, openKernel);
        cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, closeKernel);

        if(mConfig.debugView) {
            debugView("dilate", bin, [](auto&) {});
        }

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(bin, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        if(contours.size() <= 0) {
            logInfo("no contour found!!!");
        }
        // std::cout << "src have " << contours.size() << " contours." << std::endl;

        // 遍历轮廓
        double maxArea = 0;
        int maxContourIndex = -1;

        std::vector<std::vector<cv::Point>> candidateContours;
        std::vector<cv::Point> targetContour;
        std::vector<std::vector<cv::Point>> hulls;

        for(size_t i = 0; i < contours.size(); i++) {
            double contourArea = cv::contourArea(contours[i]);

            // 有子轮廓
            if(hierarchy[i][2] != -1) {
                continue;
            }

            // 凸包检测
            std::vector<cv::Point> hull;
            cv::convexHull(contours[i], hull);

            // 计算轮廓面积与轮廓凸包面积比
            double hullArea = cv::contourArea(hull);
            // logInfo(fmt::format("hull area is {:.3f}", hullArea));
            if (hullArea < 2.0) {
                continue;
            }
            double ratio = contourArea / hullArea;

            // 筛掉凸包轮廓
            if(ratio > mConfig.maxConvexHullThresh || ratio < mConfig.minConvexHullThresh) {
                continue;
            }

            hulls.push_back(hull);
            candidateContours.push_back(contours[i]);

            if(contourArea > maxArea) {
                maxArea = contourArea;
                maxContourIndex = i;
                targetContour = contours[i];
            }
        }
        if(maxContourIndex != -1) {
            cv::Rect2f roiRect = cv::boundingRect(targetContour);
            return roiRect;
        }
        return {};
    }

    std::optional<cv::Point2f> getRLabel(const cv::Mat& src, const cv::Point2f& smallCenter, const cv::Point2f& largeCenter) {
        double roiSize = euclideanDistance(smallCenter, largeCenter) * mConfig.rRoiSizeScale;
        cv::Point2f RRoiCenter = smallCenter - (smallCenter - largeCenter) * mConfig.rPosScale;
        // cv::Point2f RRoiCenter = smallCenter - (smallCenter - largeCenter) * 6.5; // true ratio

        cv::Rect2f roiRect = cv::Rect2f(RRoiCenter.x - roiSize, RRoiCenter.y - roiSize, 2 * roiSize, 2 * roiSize);
        roiRect &= cv::Rect2f(cv::Point2f(0, 0), cv::Point2f(static_cast<float>(src.cols), static_cast<float>(src.rows)));

        if(roiRect.width <= 10 || roiRect.height <= 10)
            return {};

        cv::Mat roiSrc = src(roiRect);
        // cv::Mat roiBin = binarize(roiSrc, mConfig.roiBinThresh);
        cv::Mat roiBin;
        cv::cvtColor(roiSrc, roiBin, cv::COLOR_BGR2GRAY);
        cv::threshold(roiBin, roiBin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        if(mConfig.debugView) {
            debugView("r_label", roiBin, [](auto&) {});
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(roiBin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point2f(roiRect.x, roiRect.y));

        if(contours.empty()) {
            return {};
        }

        double maxArea = 0;
        cv::RotatedRect rLabelRect;
        for(auto contour : contours) {
            for(auto point : contour) {
                if(point.x <= 0 || point.x >= roiSrc.cols || point.y <= 0 || point.y >= roiSrc.rows) {
                    break;
                }
            }
            double contouArea = cv::contourArea(contour);
            if(contouArea > maxArea) {
                rLabelRect = cv::minAreaRect(contour);
            }
        }

        return rLabelRect.center;
    }

    void detect(const cv::Mat& src, std::vector<cv::Point2f>& keyPoints) {
        auto roiRectOptional = getRuneROI(src);
        // logInfo("roi get");
        
        if(!roiRectOptional.has_value()) {
            return;
        }
        cv::Rect2f roiRect = roiRectOptional.value();

        cv::Mat roiImg = src(roiRect);

        // need to change
        cv::Mat roiBin = binarize(roiImg, mConfig.roiBinThresh);

        // auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        // cv::morphologyEx(roiBin, roiBin, cv::MORPH_CLOSE, kernel);

        if(mConfig.debugView)
            debugView("roiRect", roiBin, [](auto&) {});

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(roiBin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point2f(roiRect.x, roiRect.y));

        if(contours.size() < 2) {
            logInfo("no target contour in roiRect");
            return;
        }
        
        // logInfo("contour found in roi");

        std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a) > cv::contourArea(b);
        });
        if(mConfig.debugView) {
            debugView("targetContours", roiImg, [&](cv::Mat img) {
                cv::drawContours(img, contours, 0, cv::Scalar(255, 0, 255), 3);
                cv::drawContours(img, contours, 1, cv::Scalar(255, 0, 255), 3);
            });
        }
        std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a) > cv::contourArea(b);
        });

        cv::RotatedRect rectLarge = cv::minAreaRect(contours[0]);
        cv::RotatedRect rectSmall = cv::minAreaRect(contours[1]);

        cv::Point2f kptLarge[4];
        cv::Point2f kptSmall[4];

        rectLarge.points(kptLarge);
        rectSmall.points(kptSmall);

        // logInfo("kpt found");

        std::sort(kptLarge, kptLarge + 4, [&](const cv::Point2f& a, const cv::Point2f& b) {
            return euclideanDistance(a, rectSmall.center) < euclideanDistance(b, rectSmall.center);
        });

        std::sort(kptSmall, kptSmall + 4, [&](const cv::Point2f& a, const cv::Point2f& b) {
            return euclideanDistance(a, rectLarge.center) < euclideanDistance(b, rectLarge.center);
        });

        cv::Point2f largeRectEdge(kptLarge[0].x - kptLarge[1].x, kptLarge[0].y - kptLarge[1].y);
        cv::Point2f smallRectEdge(kptSmall[0].x - kptSmall[1].x, kptSmall[0].y - kptSmall[1].y);

        cv::Point2f largeRectEdgeCenter((kptLarge[0].x + kptLarge[1].x) / 2, (kptLarge[0].y + kptLarge[1].y) / 2);
        cv::Point2f smallRectEdgeCenter((kptSmall[0].x + kptSmall[1].x) / 2, (kptSmall[0].y + kptSmall[1].y) / 2);

        cv::Point2f large2small(largeRectEdgeCenter.x - smallRectEdgeCenter.x, largeRectEdgeCenter.y - smallRectEdgeCenter.y);

        if(crossProduct2D(large2small, largeRectEdge) > 0) {
            keyPoints.push_back(kptLarge[0]);
            keyPoints.push_back(kptLarge[1]);
        } else {
            keyPoints.push_back(kptLarge[1]);
            keyPoints.push_back(kptLarge[0]);
        }

        if(crossProduct2D(large2small, smallRectEdge) > 0) {
            keyPoints.push_back(kptSmall[0]);
            keyPoints.push_back(kptSmall[1]);
        } else {
            keyPoints.push_back(kptSmall[1]);
            keyPoints.push_back(kptSmall[0]);
        }

        // logInfo("kpt regulized");

        auto RLabelOptional = getRLabel(src, smallRectEdgeCenter, largeRectEdgeCenter);
        if(RLabelOptional.has_value()) {
            cv::Point2f RLabel = RLabelOptional.value();
            keyPoints.push_back(RLabel);
        }

        debugView("kpt", src, [&](cv::Mat img) {
            int kpt_index = 0;
            for(auto point : keyPoints) {
                cv::circle(img, point, 1, cv::Scalar(0, 255, 0), 10);
                cv::putText(img, std::to_string(kpt_index), point, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255),
                            1);
                kpt_index++;
            }
        });

    }

public:
    RuneDetector(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [this](energy_detector_control_atom, uint8_t mode, double dt) {
                     if(mode)
                         mEnable = true;
                 },
                 [&](image_frame_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame, std::string_view>);
                     ACTOR_EXCEPTION_PROBE();

                     // logInfo("Rune detector start");

                     //  if(!mEnable) {
                     //      return;
                     //  }

                     auto data = BlackBoard::instance().get<CameraFrame, std::string_view>(key).value();
                     auto frame = std::get<0>(data);

                     /**
                      * 执行代码
                      */
                     cv::Mat src = frame.frame;
                     std::vector<cv::Point2f> keyPoints;
                     detect(src, keyPoints);
                     
                     EnergyFan res;
                     res.lastUpdate = frame.lastUpdate;
                     res.cameraInfo = frame.info;
                     
                     if(keyPoints.size() == 5) {
                         res.keyPoints = keyPoints;
                     }

                     sendAll(energy_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 } };
    }
};

HUB_REGISTER_CLASS(RuneDetector);
