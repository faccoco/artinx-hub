#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "DetectedCar.hpp"
#include <caf/event_based_actor.hpp>
#include <opencv2/tracking.hpp>
#include <vector>
#include <cmath>

using MultiTrackers = std::vector<cv::Ptr<cv::TrackerKCF>>;
using VectorRect = std::vector<cv::Rect>;
using Inf = std::numeric_limits<double>;

constexpr int32_t maxNumRobot = 10;


class CarTracker final : public HubHelper<caf::event_based_actor, void, car_detect_available_atom> {
	Identifier mKey;
	MultiTrackers trackers = MultiTrackers(maxNumRobot, nullptr);
	VectorRect trackedBoxes = VectorRect(maxNumRobot, cv::Rect(0, 0, 0, 0));
	bool initialFlag = false;
	int32_t initialisedTrackerNum = 0;

public:
	CarTracker(caf::actor_config& base, const HubConfig& config)
		: HubHelper{ base, config }, mKey{ typeid(CarTracker).hash_code() } {
		for(int i = 0; i < maxNumRobot; i++) {
			trackers[i] = cv::TrackerKCF::create();
		}
	}

	caf::behavior make_behavior() override {
		return { [this](start_atom) {},
				 [&](image_frame_atom, Identifier key) { 
					if(!initialFlag) {
						CAF_LOG_ERROR("Tracker do not initialise, tracking failed!"); 
						return;
					}
					const auto data = BlackBoard::instance().get<CameraFrame>(key).value();

					DetectedCarArray carTrackedRes;
					carTrackedRes.frame = data;
					
					for(int i = 0; i < initialisedTrackerNum; ++i) {
						const auto ok = trackers[i]->update(carTrackedRes.frame.frame, trackedBoxes[i]);
                        if(ok) {
                            carTrackedRes.cars.push_back(trackedBoxes[i]);
						}
					}

					BlackBoard::instance().updateSync(mKey, std::move(carTrackedRes));
					sendAll(car_detect_available_atom_v, mKey);
				 },
                [&](car_detect_available_atom, Identifier key) {
                    const auto carDetectedRes = BlackBoard::instance().get<DetectedCarArray>(key).value();

                    DetectedCarArray carTrackedRes;
                    carTrackedRes.frame = carDetectedRes.frame;

                    if(!initialFlag) {
                        for(int i = 0; i < carDetectedRes.cars.size(); ++i) {
                            trackedBoxes[i] = carDetectedRes.cars[i];
                            carTrackedRes.cars.push_back(carDetectedRes.cars[i]);
                            trackers[i]->init(carTrackedRes.frame.frame, trackedBoxes[i]);
                            initialisedTrackerNum++;
                        }
                        initialFlag = true;

                        BlackBoard::instance().updateSync(mKey, std::move(carTrackedRes));
                        sendAll(car_detect_available_atom_v, mKey);
                    } else {
                        VectorRect trackRectRes;
                        for(int i = 0; i < initialisedTrackerNum; ++i) {
                            const auto ok = trackers[i]->update(carTrackedRes.frame.frame, trackedBoxes[i]);
                            if(ok) {
                                trackRectRes.push_back(trackedBoxes[i]);
                            }
                        }

                        const auto computeCenterPoint = [](const cv::Rect& rect) {
                            return std::make_pair(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
                        };

                        //match algorithm: Match the rectangle closest to the center
                        for(const auto& detectedRect : carDetectedRes.cars) {
                            carTrackedRes.cars.push_back(detectedRect);
                            const auto centerPoint = computeCenterPoint(detectedRect);
                            auto tmpDist = Inf::max();
                            auto matchIndex = -1;
                            for(int j = 0; j < trackRectRes.size(); ++j) {
                                const auto trackedRectCenter = computeCenterPoint(trackRectRes[j]);
                                const auto dist = std::pow((centerPoint.first - trackedRectCenter.first), 2) +
                                                  std::pow((centerPoint.second - trackedRectCenter.second), 2);
                                if(dist < tmpDist) {
                                    tmpDist = dist;
                                    matchIndex = j;
                                }
                            }
                            if(tmpDist > std::pow(carDetectedRes.cars[matchIndex].width, 2)) {
                                carTrackedRes.cars.push_back(trackRectRes[matchIndex]);
                            }
                        }

                        BlackBoard::instance().updateSync(mKey, std::move(carTrackedRes));
                        sendAll(car_detect_available_atom_v, mKey);

                        //Update
                        if(carDetectedRes.cars.size() > trackRectRes.size()) {
                            trackers.clear();
                            trackedBoxes.clear();
                            initialisedTrackerNum = 0;
                            for(const auto& detectedRect : carDetectedRes.cars) {
                                const auto tracker = cv::TrackerKCF::create();
                                trackers.push_back(tracker);
                                trackedBoxes.push_back(detectedRect);
                                tracker->init(carTrackedRes.frame.frame, trackedBoxes[initialisedTrackerNum++]);
                            }
                        }

                    }

                },

 
		};
	}

};