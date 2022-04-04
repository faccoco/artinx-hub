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

    struct ArmorData final {
        cv::Point2f armorCenter;
        cv::Point2f energyCenter;
        float angle{};
        int quadrant{};
        bool isFind = false;
    };

    ArmorData mLastData;
    bool mDirectionTested = false;
    bool mVelocityTested = false;
    int mTimes = 0;
    std::vector<ArmorData> mData;
    DetectedEnergyArray mLastRes;
    bool mEnabled = false;
    int mRotateMode = 0;

    void reset() {
        mLastData = {};
        mDirectionTested = false;
        mVelocityTested = false;
        mTimes = 0;
        mLastRes = {};
        mData.clear();
    }

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

    static bool circleLeastFit(const std::vector<cv::Point2f>& points, cv::Point2f& energyCenter) {
        if(points.size() < 3) {
            return false;
        }

        double sumX = 0.0, sumY = 0.0;
        double sumX2 = 0.0, sumY2 = 0.0;
        double sumX3 = 0.0, sumY3 = 0.0;
        double sumXy = 0.0, sum_x1y2 = 0.0, sumX2Y1 = 0.0;
        for(const auto [x, y] : points) {
            const double x2 = x * x;
            const double y2 = y * y;
            sumX += x;
            sumY += y;
            sumX2 += x2;
            sumY2 += y2;
            sumX3 += x2 * x;
            sumY3 += y2 * y;
            sumXy += x * y;
            sum_x1y2 += x * y2;
            sumX2Y1 += x2 * y;
        }
        const auto n = static_cast<double>(points.size());
        const auto pC = n * sumX2 - sumX * sumX;
        const auto pD = n * sumXy - sumX * sumY;
        const auto pE = n * sumX3 + n * sum_x1y2 - (sumX2 + sumY2) * sumX;
        const auto pG = n * sumY2 - sumY * sumY;
        const auto pH = n * sumX2Y1 + n * sumY3 - (sumX2 + sumY2) * sumY;
        const auto a = (pH * pD - pE * pG) / (pC * pG - pD * pD);
        const auto b = (pH * pC - pE * pD) / (pD * pD - pG * pC);
        // const auto c = -(a * sumX + b * sumY + sumX2 + sumY2) / n;
        const auto centerX = a * -0.5;
        const auto centerY = b * -0.5;
        // const auto radius = sqrt(a * a + b * b - 4 * c) / 2;
        energyCenter = cv::Point2f(static_cast<float>(centerX), static_cast<float>(centerY));
        return true;
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

    static double getDistance(const cv::Point2f& a, const cv::Point2f& b) {
        return hypot(a.x - b.x, a.y - b.y);
    }

    bool armorJudge(const std::vector<cv::Point>& contour, const cv::RotatedRect& rotatedRect) {
        cv::Point2f rectPoints[4];
        rotatedRect.points(rectPoints);
        const auto height = std::min(rotatedRect.size.height, rotatedRect.size.width);
        const auto width = std::max(rotatedRect.size.height, rotatedRect.size.width);
        const auto area = contourArea(contour);
        std::vector<cv::Point2f> rectContour;

        for(auto& rectPoint : rectPoints) {
            rectContour.push_back(rectPoint);
        }
        if(const auto match = matchShapes(contour, rectContour, cv::CONTOURS_MATCH_I1, 0.0); area > mConfig.armorMinArea &&
           area < mConfig.armorMaxArea && width / height < mConfig.armorMaxWHRatio && width / height > mConfig.armorMinWHRatio &&
           contourArea(contour) / rotatedRect.size.area() > mConfig.armorMinAreaRatio && match < 0.3)
            return true;
        return false;
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

    static bool changeAngle(const int quadrant, const float angle, float& tranAngle) {
        if(quadrant == 1) {
            tranAngle = angle;
        } else if(quadrant == 2) {
            tranAngle = 90 + 90 - angle;
        } else if(quadrant == 3) {
            tranAngle = 180 + angle;
        } else if(quadrant == 4) {
            tranAngle = 270 + 90 - angle;
        } else {
            logInfo("Quadrant = 0");
            return false;
        }
        return true;
    }

    static float angleCalculate(ArmorData data1, ArmorData data2, const bool direction) {
        if(direction) {
            if(data1.quadrant == data2.quadrant) {
                if(data1.quadrant == 1 || data1.quadrant == 3) {
                    return data1.angle - data2.angle;
                } else
                    return data2.angle - data1.angle;
            } else if(data1.quadrant == 1 || data1.quadrant == 3) {
                return data1.angle + data2.angle;
            } else if(data1.quadrant == 2 || data1.quadrant == 4) {
                return 180 - data1.angle - data2.angle;
            }
        } else {
            if(data1.quadrant == data2.quadrant) {
                if(data1.quadrant == 1 || data1.quadrant == 3) {
                    return data2.angle - data1.angle;
                } else
                    return data1.angle - data2.angle;
            } else if(data1.quadrant == 1 || data1.quadrant == 3) {
                return 180 - data1.angle - data2.angle;
            } else if(data1.quadrant == 2 || data1.quadrant == 4) {
                return data1.angle + data2.angle;
            }
        }
    }

    static constexpr int frames = 20;
    bool getDirection(DetectedEnergyArray& detectedEnergyArray) {
        if(mTimes < frames && mLastData.isFind) {
            mData.push_back(mLastData);
            mTimes++;
            return false;
        } else {
            float angles[frames];
            int negative = 0;
            int positive = 0;
            if(static_cast<int>(mData.size()) != frames) {
                mTimes = 0;
                mData.clear();
                return false;
            }
            for(int i = 0; i < frames; ++i) {
                changeAngle(mData[i].quadrant, mData[i].angle, angles[i]);
            }
            for(int j = 1; j < 3; ++j) {
                for(int i = 0; i < frames - j; ++i) {
                    if((angles[i] - angles[i + j]) > 0 || (angles[i] - angles[i + j]) < -300) {
                        positive++;
                    } else if((angles[i] - angles[i + j]) < 0 || (angles[i] - angles[i + j]) > 300) {
                        negative++;
                    }
                }
            }
            if(positive > negative) {
                detectedEnergyArray.direction = true;
                logInfo("Clockwise");
            } else if(positive < negative) {
                detectedEnergyArray.direction = false;
                logInfo("Anticlockwise");
            } else {
                logInfo("Direction detecting failed");
                return false;
            }
            mTimes = 0;
            mDirectionTested = true;
            mData.clear();
            return true;
        }
    }

    bool getArmorCenter(const cv::Mat& src, ArmorData& data) {
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
        debugView("selected_contour", src, [&](cv::Mat& img) {
            cv::drawContours(img, armorContours, static_cast<int>(index), cv::Scalar{ 255, 0, 0 });
        });

        bool findArmor = false;
        const auto finalRect = boundingRect(armorContours[index]);
        const auto finalROI = binary(finalRect);
        const auto moments = cv::moments(finalROI, true);
        debugView("finalROI", finalROI, [&](cv::Mat& img) {
            const auto centerX = moments.m10 / moments.m00;
            const auto centerY = moments.m01 / moments.m00;

            cv::circle(img, { static_cast<int>(centerX), static_cast<int>(centerY) }, 5.0, cv::Scalar{ 0 });

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
            if(dx * std::cos(theta) + dy * std::sin(theta) < 0.0)
                theta += glm::pi<double>();
            const auto cos = std::cos(theta), sin = std::sin(theta);

            cv::line(img, { static_cast<int>(centerX), static_cast<int>(centerY) },
                     { static_cast<int>(centerX + 50.0 * cos), static_cast<int>(centerY + 50.0 * sin) }, cv::Scalar{ 0 });
        });

        cv::RotatedRect finalSqua;
        double maxArea = 0;
        std::vector<std::vector<cv::Point> > finalContours;
        std::vector<cv::Vec4i> finalHierarchy;

        findContours(finalROI, finalContours, finalHierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, finalRect.tl());
        for(size_t i = 0; i < finalContours.size(); ++i) {
            if(finalHierarchy[i][3] != -1) {
                cv::RotatedRect squa = minAreaRect(finalContours[i]);
                if(armorJudge(finalContours[i], squa)) {
                    if(const auto area = contourArea(finalContours[i]); area > maxArea) {
                        maxArea = area;
                        finalSqua = squa;
                        findArmor = true;
                    }
                }
            }
        }
        if(!findArmor) {
            logInfo("Armor detect failed");
            return false;
        }

        data.armorCenter = finalSqua.center + mConfig.offset;
        const auto finalRrect = minAreaRect(armorContours[index]);
        const auto arrowCenter = finalRrect.center + mConfig.offset;

        if(const auto minVal = std::min(finalSqua.size.height, finalSqua.size.width);
           getDistance(arrowCenter, data.armorCenter) < minVal * 0.8) {
            data.isFind = false;
        } else {
            float tranAngle;
            if(finalSqua.size.width > finalSqua.size.height) {
                tranAngle = 90 - fabs(finalSqua.angle);
            } else {
                tranAngle = fabs(finalSqua.angle);
            }
            data.angle = tranAngle;
            if(tranAngle < 20) {
                if(finalSqua.size.width > finalSqua.size.height && arrowCenter.x < data.armorCenter.x) {
                    data.quadrant = 1;
                } else if(finalSqua.size.width < finalSqua.size.height && arrowCenter.x > data.armorCenter.x) {
                    data.quadrant = 2;
                } else if(finalSqua.size.width > finalSqua.size.height && arrowCenter.x > data.armorCenter.x) {
                    data.quadrant = 3;
                } else if(finalSqua.size.width < finalSqua.size.height && arrowCenter.x < data.armorCenter.x) {
                    data.quadrant = 4;
                }
            } else if(tranAngle > 70) {
                if(finalSqua.size.width > finalSqua.size.height && arrowCenter.y > data.armorCenter.y) {
                    data.quadrant = 1;
                } else if(finalSqua.size.width < finalSqua.size.height && arrowCenter.y > data.armorCenter.y) {
                    data.quadrant = 2;
                } else if(finalSqua.size.width > finalSqua.size.height && arrowCenter.y < data.armorCenter.y) {
                    data.quadrant = 3;
                } else if(finalSqua.size.width < finalSqua.size.height && arrowCenter.y < data.armorCenter.y) {
                    data.quadrant = 4;
                }
            } else {
                if(arrowCenter.x < data.armorCenter.x && arrowCenter.y >= data.armorCenter.y &&
                   finalSqua.size.width > finalSqua.size.height) {
                    data.quadrant = 1;
                } else if(arrowCenter.x >= data.armorCenter.x && arrowCenter.y > data.armorCenter.y &&
                          finalSqua.size.width <= finalSqua.size.height) {
                    data.quadrant = 2;
                } else if(arrowCenter.x > data.armorCenter.x && arrowCenter.y <= data.armorCenter.y &&
                          finalSqua.size.width > finalSqua.size.height) {
                    data.quadrant = 3;
                } else if(arrowCenter.x <= data.armorCenter.x && arrowCenter.y < data.armorCenter.y &&
                          finalSqua.size.width <= finalSqua.size.height) {
                    data.quadrant = 4;
                }
            }
            const auto angle = glm::radians(data.angle);
            const auto cosAngle = std::cos(angle), sinAngle = std::sin(angle);

            if(data.quadrant == 1) {
                data.energyCenter.x = data.armorCenter.x - mConfig.radius * cosAngle;
                data.energyCenter.y = data.armorCenter.y + mConfig.radius * sinAngle;
            } else if(data.quadrant == 2) {
                data.energyCenter.x = data.armorCenter.x + mConfig.radius * cosAngle;
                data.energyCenter.y = data.armorCenter.y + mConfig.radius * sinAngle;
            } else if(data.quadrant == 3) {
                data.energyCenter.x = data.armorCenter.x + mConfig.radius * cosAngle;
                data.energyCenter.y = data.armorCenter.y - mConfig.radius * sinAngle;
            } else if(data.quadrant == 4) {
                data.energyCenter.x = data.armorCenter.x - mConfig.radius * cosAngle;
                data.energyCenter.y = data.armorCenter.y - mConfig.radius * sinAngle;
            }
            data.isFind = true;
        }

        return true;
    }

    bool predict(const ArmorData& data, cv::Point2f& preCenter, const int predictMode, const int direction) {
        if(predictMode == 0) {
            static int count = 0;
            static std::vector<cv::Point2f> armorPoints;
            armorPoints.resize(50);
            if(count < 50) {
                armorPoints.insert(armorPoints.begin() + count, data.armorCenter);
                count++;
                return false;
            } else if(count == 50) {
                cv::Point2f center;
                circleLeastFit(armorPoints, center);
                float preAngle;
                if(direction == 0) {
                    preAngle = mConfig.predictAngle;
                } else {
                    preAngle = -mConfig.predictAngle;
                }
                const auto x = data.armorCenter.x - center.x;
                const auto y = data.armorCenter.y - center.y;
                preCenter.x = x * cos(preAngle) + y * sin(preAngle) + center.x;
                preCenter.y = -x * sin(preAngle) + y * cos(preAngle) + center.y;

                return true;
            }
        } else if(predictMode == 1) {
            if(data.energyCenter == cv::Point2f(0, 0)) {
                return false;
            }

            float preAngle;
            if(direction == 0) {
                preAngle = mConfig.predictAngle;
            } else {
                preAngle = -mConfig.predictAngle;
            }
            const auto x = data.armorCenter.x - data.energyCenter.x;
            const auto y = data.armorCenter.y - data.energyCenter.y;
            preCenter.x = x * cos(preAngle) + y * sin(preAngle) + data.energyCenter.x;
            preCenter.y = -x * sin(preAngle) + y * cos(preAngle) + data.energyCenter.y;

            return true;

        } else if(predictMode == 2) {
            const auto preAngle = mConfig.predictAngle;
            const auto dis = mConfig.radius * tan(preAngle);
            const auto dis_x = dis * sin(glm::radians(data.angle));
            const auto dis_y = dis * cos(glm::radians(data.angle));

            cv::Point2f tangent;
            if(direction == 0) {

                if(data.quadrant == 1) {
                    tangent.x = data.armorCenter.x - dis_x;
                    tangent.y = data.armorCenter.y - dis_y;
                } else if(data.quadrant == 2) {
                    tangent.x = data.armorCenter.x - dis_x;
                    tangent.y = data.armorCenter.y + dis_y;
                } else if(data.quadrant == 3) {
                    tangent.x = data.armorCenter.x + dis_x;
                    tangent.y = data.armorCenter.y + dis_y;
                } else if(data.quadrant == 4) {
                    tangent.x = data.armorCenter.x + dis_x;
                    tangent.y = data.armorCenter.y - dis_y;
                } else {
                    return false;
                }

                const auto x = tangent.x - data.armorCenter.x;
                const auto y = tangent.y - data.armorCenter.y;
                preCenter.x = x * cos(preAngle / 2) + y * sin(preAngle / 2) + data.armorCenter.x;
                preCenter.y = -x * sin(preAngle / 2) + y * cos(preAngle / 2) + data.armorCenter.y;
            } else {
                if(data.quadrant == 1) {
                    tangent.x = data.armorCenter.x + dis_x;
                    tangent.y = data.armorCenter.y + dis_y;
                } else if(data.quadrant == 2) {
                    tangent.x = data.armorCenter.x + dis_x;
                    tangent.y = data.armorCenter.y - dis_y;
                } else if(data.quadrant == 3) {
                    tangent.x = data.armorCenter.x - dis_x;
                    tangent.y = data.armorCenter.y - dis_y;
                } else if(data.quadrant == 4) {
                    tangent.x = data.armorCenter.x - dis_x;
                    tangent.y = data.armorCenter.y + dis_y;
                } else {
                    return false;
                }

                const auto x = tangent.x - data.armorCenter.x;
                const auto y = tangent.y - data.armorCenter.y;
                preCenter.x = x * cos(-preAngle / 2) + y * sin(-preAngle / 2) + data.armorCenter.x;
                preCenter.y = -x * sin(-preAngle / 2) + y * cos(-preAngle / 2) + data.armorCenter.y;
            }
            return true;
        }

        return false;
    }

    DetectedEnergyArray detect(const CameraFrame& input) {
        DetectedEnergyArray res;
        if(mRotateMode == 0) {
            ArmorData armorData;
            if(!getArmorCenter(input.frame, armorData)) {
                res.predictPoint = cv::Point2f(0, 0);
            } else if(mDirectionTested) {
                cv::Point2f preCenter;
                predict(armorData, preCenter, mConfig.smallPredictMode, mLastRes.direction);
                res.predictPoint = preCenter;
            }
            mLastData = armorData;
        } else if(mRotateMode == 1) {
            ArmorData armordata;
            if(!getArmorCenter(input.frame, armordata)) {
                res.predictPoint = cv::Point2f(0, 0);
            } else if(mDirectionTested) {
                cv::Point2f preCenter;
                if(!predict(armordata, preCenter, mConfig.bigPredictMode, mLastRes.direction)) {
                    res.predictPoint = cv::Point2f(0, 0);
                } else {
                    res.predictPoint = preCenter;
                }
            }
            mLastData = armordata;
        }
        if(!mDirectionTested) {
            getDirection(res);
        }
        return res;
    }

    bool velocityCalculate(const DetectedEnergyArray& detectEnergyArray) {
        // FIXME: static?
        const int frameNums = 50;
        static int times = 0;
        static std::vector<ArmorData> datas;
        float circleAngle[frameNums];
        datas.resize(frameNums);
        if(times < frameNums && mLastData.isFind) {
            datas.insert(datas.begin() + times, mLastData);
            times++;
            return false;
        } else {
            if(static_cast<int>(datas.size()) != frameNums) {
                times = 0;
                datas.clear();
                return false;
            }
            if(times == frameNums) {
                for(int i = 0; i < 149; ++i) {
                    circleAngle[i] = angleCalculate(datas[i], datas[i + 1], detectEnergyArray.direction);
                }
                mVelocityTested = true;
                return false;
            }
            times = 0;
            mDirectionTested = true;
            datas.clear();
            return true;
        }
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
                     mEnabled = enable;
                     mRotateMode = mode;
                     reset();
                 },
                 [&](image_frame_atom, Identifier key) {
                     if(!mEnabled)
                         return;
                     auto data = BlackBoard::instance().get<CameraFrame>(key).value();
                     mLastRes = detect(data);
                     if(mLastRes.predictPoint.x == 0.0f)
                         return;
                     std::cout << "Success" << std::endl;
                     BlackBoard::instance().updateSync(mKey, mLastRes);
                     sendAll(energy_detect_available_atom_v, mKey);

                     // only for test
                     const auto img = data.frame.clone();
                     cv::circle(img, { static_cast<int>(mLastRes.predictPoint.x), static_cast<int>(mLastRes.predictPoint.y) }, 10,
                                cv::Scalar{ 255, 0, 0 });
                     data.frame = img;
                     BlackBoard::instance().updateSync(mKey, data);
                 } };
    }
};

HUB_REGISTER_CLASS(EnergyDetector);
