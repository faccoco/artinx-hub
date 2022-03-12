#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "DetectedCar.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>
#include <limits>
#include <opencv2/tracking.hpp>
#include <fmt/format.h>
#include <Utility.hpp>
#include <vector>

using MultiTrackers = std::vector<cv::Ptr<cv::TrackerKCF>>;
using VectorRect = std::vector<cv::Rect>;

constexpr double infinity = std::numeric_limits<double>::infinity();
constexpr uint32_t maxNumRobot = 10;
constexpr double iouThreshold = 0.5;

class CarTracker final : public HubHelper<caf::event_based_actor, void, car_detect_available_atom> {
    Identifier mKey;
    MultiTrackers mTrackers = MultiTrackers(maxNumRobot, nullptr);
    VectorRect mTrackedBoxes = VectorRect(maxNumRobot, cv::Rect(0, 0, 0, 0));
    bool mInitialFlag = false;
    int32_t mInitialisedTrackerNum = 0;

    std::vector<double> computeIoUAdjacencyMat(const VectorRect& detectRectRes, const VectorRect& trackRectRes) {
        std::vector<double> res(trackRectRes.size() * detectRectRes.size());
        const auto computeIoU = [](const cv::Rect& rect1, const cv::Rect rect2) {
            auto distX = abs(rect1.x - rect2.x);
            auto width = (rect1.x < rect2.x ? rect1.width : rect2.width) - distX;
            auto distY = abs(rect1.y - rect2.y);
            auto height = (rect1.y < rect2.y ? rect1.height : rect2.height) - distY;
            auto areaIoU = width * height;
            if(width < 0 && height < 0) {
                areaIoU *= -1;
            }
            return areaIoU;
        };
        for(int i = 0; i < trackRectRes.size(); ++i) {
            for(int j = 0; j < detectRectRes.size(); ++j) {
                res[i * detectRectRes.size() + j] = computeIoU(trackRectRes[i], detectRectRes[j]) + 1e6;
            } 
        }
        return res;
    }

public:
    CarTracker(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(CarTracker).hash_code() } {
        for(int i = 0; i < maxNumRobot; i++) {
            mTrackers[i] = cv::TrackerKCF::create();
        }
    }

    caf::behavior make_behavior() override {
        return {
            [this](start_atom) {},
            [&](image_frame_atom, Identifier key) {
                if(!mInitialFlag) {
                    return;
                }
                const auto data = BlackBoard::instance().get<CameraFrame>(key).value();

                const auto t1 = Clock::now();
                DetectedCarArray carTrackedRes;
                carTrackedRes.frame = data;

                for(int i = 0; i < mInitialisedTrackerNum; ++i) {
                    const auto ok = mTrackers[i]->update(carTrackedRes.frame.frame, mTrackedBoxes[i]);
                    if(ok) {
                        carTrackedRes.cars.push_back(mTrackedBoxes[i]);
                    }
                }
                const auto t2 = Clock::now();
                CAF_LOG_INFO(
                    fmt::format("image_frame_atom:track time {:.4f}s", (t2 - t1).count() / 1e9));
                BlackBoard::instance().updateSync(mKey, std::move(carTrackedRes));
                sendAll(car_detect_available_atom_v, mKey);
            },
            [&](car_detect_available_atom, Identifier key) {

                const auto carDetectedRes = BlackBoard::instance().get<DetectedCarArray>(key).value();

                const auto t1 = Clock::now();
                DetectedCarArray carTrackedRes;
                carTrackedRes.cars = carDetectedRes.cars;
                carTrackedRes.frame = carDetectedRes.frame;

                if(!mInitialFlag) {
                    for(int i = 0; i < carDetectedRes.cars.size(); ++i) {
                        mTrackedBoxes[i] = carDetectedRes.cars[i];
                        mTrackers[i]->init(carTrackedRes.frame.frame, mTrackedBoxes[i]);
                        mInitialisedTrackerNum++;
                    }
                    mInitialFlag = true;

                    BlackBoard::instance().updateSync(mKey, std::move(carTrackedRes));
                    sendAll(car_detect_available_atom_v, mKey);
                } else {
                    VectorRect trackRectRes;
                    std::vector<uint32_t> trackRectIndex;
                    for(int i = 0; i < mInitialisedTrackerNum; ++i) {
                        const auto ok = mTrackers[i]->update(carTrackedRes.frame.frame, mTrackedBoxes[i]);
                        if(ok) {
                            trackRectRes.push_back(mTrackedBoxes[i]);
                            trackRectIndex.push_back(i);
                        }
                    }
                    if(trackRectRes.size() >= carDetectedRes.cars.size()) {
                        const auto iouAdjacencyMat = computeIoUAdjacencyMat(carDetectedRes.cars, trackRectRes);
                        const auto matchResult = solveKM(carDetectedRes.cars.size(), trackRectRes.size(), iouAdjacencyMat);

                        for(int i = 0; i < trackRectRes.size(); ++i) {
                            if(matchResult[i] < carDetectedRes.cars.size()) {
                                if(iouAdjacencyMat[matchResult[i] * trackRectRes.size() + i] >
                                    iouThreshold * trackRectRes[i].area() + 1e6) {
                                    mTrackedBoxes[trackRectIndex[i]] = carDetectedRes.cars[matchResult[i]];
                                    continue;
                                }
                            }
                            carTrackedRes.cars.push_back(trackRectRes[i]);
                        }
                    } else {
                        mTrackers.clear();
                        mTrackedBoxes.clear();
                        mInitialisedTrackerNum = 0;
                        for(const auto& detectedRect : carDetectedRes.cars) {
                            mTrackers.push_back(cv::TrackerKCF::create());
                            mTrackedBoxes.push_back(detectedRect);
                            mTrackers[mInitialisedTrackerNum]->init(carTrackedRes.frame.frame, mTrackedBoxes[mInitialisedTrackerNum]);
                            mInitialisedTrackerNum++;
                        }
                    
                    }

                    const auto t2 = Clock::now();
                    CAF_LOG_INFO(fmt::format("carDetect_atom:track time {:.4f}s", (t2 - t1).count() / 1e9));
                    BlackBoard::instance().updateSync(mKey, std::move(carTrackedRes));
                    sendAll(car_detect_available_atom_v, mKey);

                }
            },

        };
    }
};

HUB_REGISTER_CLASS(CarTracker);
