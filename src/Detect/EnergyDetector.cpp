#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "EnergyDetect.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <fmt/format.h>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

struct EnergyDetectorSettings final {
    int color;
    int rotateMode;
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
    return f.object(x).fields(f.field("rotateMode", x.rotateMode), f.field("smallPredictMode", x.smallPredictMode),
                              f.field("bigPredictMode", x.bigPredictMode), f.field("armorMinArea", x.armorMinArea),
                              f.field("armorMaxArea", x.armorMaxArea), f.field("armorMinWHRatio", x.armorMinWHRatio),
                              f.field("stripMaxWHRatio", x.stripMaxWHRatio), f.field("stripMaxAreaRatio", x.stripMaxAreaRatio),
                              f.field("noiseArea", x.noiseArea), f.field("predictAngle", x.predictAngle),
                              f.field("radius", x.radius), f.field("offsetX", x.offset.x), f.field("offsetY", x.offset.y));
}

class EnergyDetector final : public HubHelper<caf::event_based_actor, EnergyDetectorSettings, energy_detect_available_atom> {

    Identifier mKey;

    struct ArmorData final {
        cv::Point2f armorCenter;
        cv::Point2f energyCenter;
        float angle;
        int quadrant;
        bool isFind = false;
    };

    ArmorData mLastData;
    bool mDirectionTested = false;
    bool mVelocityTested = false;

    bool circleLeastFit(const std::vector<cv::Point2f>& points, cv::Point2f& energyCenter) {
        float centerX = 0.0f;
        float centerY = 0.0f;
        float radius = 0.0f;

        if(points.size() < 3) {
            return false;
        }
        double sumX = 0.0f, sumY = 0.0f;
        double sumX2 = 0.0f, sumY2 = 0.0f;
        double sumX3 = 0.0f, sumY3 = 0.0f;
        double sumXy = 0.0f, sum_x1y2 = 0.0f, sumX2Y1 = 0.0f;
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
        const double n = points.size();
        const auto pC = n * sumX2 - sumX * sumX;
        const auto pD = n * sumXy - sumX * sumY;
        const auto pE = n * sumX3 + n * sum_x1y2 - (sumX2 + sumY2) * sumX;
        const auto pG = n * sumY2 - sumY * sumY;
        const auto pH = n * sumX2Y1 + n * sumY3 - (sumX2 + sumY2) * sumY;
        const auto a = (pH * pD - pE * pG) / (pC * pG - pD * pD);
        const auto b = (pH * pC - pE * pD) / (pD * pD - pG * pC);
        const auto c = -(a * sumX + b * sumY + sumX2 + sumY2) / n;
        centerX = a / (-2);
        centerY = b / (-2);
        radius = sqrt(a * a + b * b - 4 * c) / 2;
        energyCenter = cv::Point2f(centerX, centerY);
        return true;
    }

    bool setBinary(const cv::Mat src, cv::Mat& binary, const int colorMode) {
        std::vector<cv::Mat> imgChannels;
        split(binary, imgChannels);

        if(colorMode == 0) {
            const auto energyRed = imgChannels[2] - imgChannels[0];
            threshold(energyRed, binary, 100, 255, cv::THRESH_BINARY);
        } else if(colorMode == 1) {
            const auto energyBlue = imgChannels[0] - imgChannels[2];
            threshold(energyBlue, binary, 100, 255, cv::THRESH_BINARY);
        } else {
            logInfo("Binary failed \n");
            return false;
        }
        return true;
    }

    double getDistance(const cv::Point2f& a, const cv::Point2f& b) {
        return hypot(a.x - b.x, a.y - b.y);
    }

    bool armorJudge(const std::vector<cv::Point>& contour, const cv::RotatedRect& rotatedRect,
                    const EnergyDetectorSettings& settings) {
        cv::Point2f rectPoints[4];
        rotatedRect.points(rectPoints);
        const auto height = std::min(rotatedRect.size.height, rotatedRect.size.width);
        const auto width = std::max(rotatedRect.size.height, rotatedRect.size.width);
        const auto area = contourArea(contour);
        std::vector<cv::Point2f> rectContour;

        for(auto& rectPoint : rectPoints) {
            rectContour.push_back(rectPoint);
        }
        if(const auto match = matchShapes(contour, rectContour, cv::CONTOURS_MATCH_I1, 0.0); area > settings.armorMinArea &&
           area < settings.armorMaxArea && width / height < settings.armorMaxWHRatio &&
           width / height > settings.armorMinWHRatio &&
           contourArea(contour) / rotatedRect.size.area() > settings.armorMinAreaRatio && match < 0.3)
            return true;
        return false;
    }

    bool stripJudge(const std::vector<cv::Point>& contour, const cv::RotatedRect& rotatedRect,
                    const EnergyDetectorSettings& settings) {
        cv::Point2f rectPoints[4];
        rotatedRect.points(rectPoints);
        const double height = std::min(rotatedRect.size.height, rotatedRect.size.width);
        const double width = std::max(rotatedRect.size.height, rotatedRect.size.width);
        double area = contourArea(contour);

        if(height * width > settings.stripMinArea && height * width < settings.stripMaxArea &&
           width / height < settings.stripMaxWHRatio && width / height > settings.stripMinWHRatio &&
           contourArea(contour) / rotatedRect.size.area() < settings.stripMaxAreaRatio)
            return true;
        return false;
    }

    bool changeAngle(const int quadrant, const float angle, float& tranAngle) {
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

    float angleCalculate(ArmorData data1, ArmorData data2, const bool direction) {
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
        } else if(!direction) {
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
    int mTimes = 0;
    std::vector<ArmorData> mData;
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

    bool getArmorCenter(const cv::Mat src, const EnergyDetectorSettings& settings, ArmorData& data) {
        auto binary = src.clone();
        setBinary(src, binary, settings.color);
        auto element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        dilate(binary, binary, element);
        element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));
        erode(binary, binary, element);

        std::vector<std::vector<cv::Point> > armorContours;
        std::vector<cv::Vec4i> armorHierarchy;
        findContours(binary, armorContours, armorHierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
        const auto armorContoursSize = armorContours.size();
        if(armorContoursSize == 0) {
            logInfo("Energy detect failed");
            return false;
        }

        std::vector<uint32_t> conIndices;
        for(uint32_t i = 0; i < armorContoursSize; ++i) {
            if(contourArea(armorContours[i]) > settings.noiseArea && armorHierarchy[i][3] == -1) {
                if(stripJudge(armorContours[i], minAreaRect(armorContours[i]), settings)) {
                    conIndices.push_back(i);
                }
            }
        }

        if(conIndices.empty()) {
            logInfo("Strip detect failed: no strip");
            return false;
        }

        uint32_t index = std::numeric_limits<uint32_t>::max();
        double minScore = 1e10;

        for(const auto conIndex : conIndices) {
            const auto finalLength = arcLength(armorContours[conIndex], true);
            const auto finalArea = contourArea(armorContours[conIndex]);

            if(const auto score = finalArea + finalLength * 10; score < minScore) {
                minScore = score;
                index = conIndex;
            }
        }
        if(index == std::numeric_limits<uint32_t>::max()) {
            logInfo("Strip detect failed: no strip contour \n");
            return false;
        }

        bool findArmor = false;
        const auto finalRect = boundingRect(armorContours[index]);
        const auto finalROI = binary(finalRect);
        cv::RotatedRect finalSqua;
        double maxArea = 0;
        std::vector<std::vector<cv::Point> > finalContours;
        std::vector<cv::Vec4i> finalHierarchy;

        findContours(finalROI, finalContours, finalHierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, finalRect.tl());
        for(size_t i = 0; i < finalContours.size(); ++i) {
            if(finalHierarchy[i][3] != -1) {
                cv::RotatedRect squa = minAreaRect(finalContours[i]);
                if(armorJudge(finalContours[i], squa, settings)) {
                    if(const auto area = contourArea(finalContours[i]); area > maxArea) {
                        maxArea = area;
                        finalSqua = squa;
                        findArmor = true;
                    }
                }
            }
        }
        if(findArmor == false) {
            logInfo("Armor detect failed");
            return false;
        }
        data.armorCenter = finalSqua.center + settings.offset;
        const auto finalRrect = minAreaRect(armorContours[index]);
        const auto arrowCenter = finalRrect.center + settings.offset;

        if(const auto minVal = std::min(finalSqua.size.height, finalSqua.size.width);
           getDistance(arrowCenter, data.armorCenter) < minVal * 0.8) {
            data.isFind = false;
        } else {
            float tranAngle = 0.0;
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
            if(data.quadrant == 1) {
                data.energyCenter.x = data.armorCenter.x - settings.radius * cos(data.angle * CV_PI / 180);
                data.energyCenter.y = data.armorCenter.y + settings.radius * sin(data.angle * CV_PI / 180);
            } else if(data.quadrant == 2) {
                data.energyCenter.x = data.armorCenter.x + settings.radius * cos(data.angle * CV_PI / 180);
                data.energyCenter.y = data.armorCenter.y + settings.radius * sin(data.angle * CV_PI / 180);
            } else if(data.quadrant == 3) {
                data.energyCenter.x = data.armorCenter.x + settings.radius * cos(data.angle * CV_PI / 180);
                data.energyCenter.y = data.armorCenter.y - settings.radius * sin(data.angle * CV_PI / 180);
            } else if(data.quadrant == 4) {
                data.energyCenter.x = data.armorCenter.x - settings.radius * cos(data.angle * CV_PI / 180);
                data.energyCenter.y = data.armorCenter.y - settings.radius * sin(data.angle * CV_PI / 180);
            }
            data.isFind = true;
        }

        return true;
    }

    bool predict(const ArmorData data, cv::Point2f& preCenter, const float predictAngle, const int predictMode,
                 const int direction, const int radius) {
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
                    preAngle = predictAngle;
                } else {
                    preAngle = -predictAngle;
                }
                double x = data.armorCenter.x - center.x;
                double y = data.armorCenter.y - center.y;
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
                preAngle = predictAngle;
            } else {
                preAngle = -predictAngle;
            }
            const auto x = data.armorCenter.x - data.energyCenter.x;
            const auto y = data.armorCenter.y - data.energyCenter.y;
            preCenter.x = x * cos(preAngle) + y * sin(preAngle) + data.energyCenter.x;
            preCenter.y = -x * sin(preAngle) + y * cos(preAngle) + data.energyCenter.y;

            return true;

        } else if(predictMode == 2) {
            const auto preAngle = predictAngle;
            const auto dis = radius * tan(preAngle);
            const auto dis_x = dis * sin(data.angle * CV_PI / 180);
            const auto dis_y = dis * cos(data.angle * CV_PI / 180);

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

    void detect(const DetectedEnergyArray& inputDetectEnergyArray, DetectedEnergyArray outputDetectEnergyArray,
                const EnergyDetectorSettings& settings) {

        if(settings.rotateMode == 0) {
            ArmorData armorData;
            if(getArmorCenter(inputDetectEnergyArray.frame.frame, settings, armorData) == false) {
                outputDetectEnergyArray.predictPoint = cv::Point2f(0, 0);
            } else if(mDirectionTested) {
                cv::Point2f preCenter;
                predict(armorData, preCenter, settings.predictAngle, settings.smallPredictMode, inputDetectEnergyArray.direction,
                        settings.radius);
            }
            mLastData = armorData;
        } else if(settings.rotateMode == 1) {
            ArmorData armordata;
            if(getArmorCenter(inputDetectEnergyArray.frame.frame, settings, armordata) == false) {
                outputDetectEnergyArray.predictPoint = cv::Point2f(0, 0);
            } else if(mDirectionTested) {
                cv::Point2f preCenter;
                if(predict(armordata, preCenter, settings.predictAngle, settings.bigPredictMode, inputDetectEnergyArray.direction,
                           settings.radius) == false) {
                    outputDetectEnergyArray.predictPoint = cv::Point2f(0, 0);
                } else {
                    outputDetectEnergyArray.predictPoint = preCenter;
                }
            }
            mLastData = armordata;
        }
        if(!mDirectionTested) {
            getDirection(outputDetectEnergyArray);
        }
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
            mDirectionTested = 1;
            datas.clear();
            return true;
        }
    }

public:
    EnergyDetector(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(EnergyDetector).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](update_posture_atom, Identifier key) {
                     const auto settings = BlackBoard::instance().get<EnergyDetectorSettings>(key).value();
                     auto data = BlackBoard::instance().get<DetectedEnergyArray>(key).value();
                     DetectedEnergyArray res;
                     detect(data, res, settings);
                     BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(energy_detect_available_atom_v, mKey);
                 } };
    }
};

HUB_REGISTER_CLASS(EnergyDetector);
