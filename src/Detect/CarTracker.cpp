#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "DetectedCar.hpp"
#include <caf/event_based_actor.hpp>
#include <opencv2/tracking.hpp>
#include <vector>
#include <limits>

using MultiTrackers = std::vector<cv::Ptr<cv::TrackerKCF>>;
using VectorRect = std::vector<cv::Rect>;

constexpr double infinity = std::numeric_limits<double>::infinity();
constexpr int32_t maxNumRobot = 10;


class CarTracker final : public HubHelper<caf::event_based_actor, void, car_detect_available_atom> {
	Identifier mKey;
	MultiTrackers mTrackers = MultiTrackers(maxNumRobot, nullptr);
	VectorRect mTrackedBoxes = VectorRect(maxNumRobot, cv::Rect(0, 0, 0, 0));
	bool mInitialFlag = false;
	int32_t mInitialisedTrackerNum = 0;

public:
	CarTracker(caf::actor_config& base, const HubConfig& config)
		: HubHelper{ base, config }, mKey{ typeid(CarTracker).hash_code() } {
		for(int i = 0; i < maxNumRobot; i++) {
			mTrackers[i] = cv::TrackerKCF::create();
		}
	}

	caf::behavior make_behavior() override {
		return { [this](start_atom) {},
				 [&](image_frame_atom, Identifier key) { 
					if(!mInitialFlag) {
						return;
					}
					const auto data = BlackBoard::instance().get<CameraFrame>(key).value();

					DetectedCarArray carTrackedRes;
					carTrackedRes.frame = data;
					
					for(int i = 0; i < mInitialisedTrackerNum; ++i) {
						const auto ok = mTrackers[i]->update(carTrackedRes.frame.frame, mTrackedBoxes[i]);
                        if(ok) {
                            carTrackedRes.cars.push_back(mTrackedBoxes[i]);
						}
					}

					BlackBoard::instance().updateSync(mKey, std::move(carTrackedRes));
					sendAll(car_detect_available_atom_v, mKey);
				 },
                [&](car_detect_available_atom, Identifier key) {
                    const auto carDetectedRes = BlackBoard::instance().get<DetectedCarArray>(key).value();

                    DetectedCarArray carTrackedRes;
                    carTrackedRes.frame = carDetectedRes.frame;

                    if(!mInitialFlag) {
                        for(int i = 0; i < carDetectedRes.cars.size(); ++i) {
                            mTrackedBoxes[i] = carDetectedRes.cars[i];
                            carTrackedRes.cars.push_back(carDetectedRes.cars[i]);
                            mTrackers[i]->init(carTrackedRes.frame.frame, mTrackedBoxes[i]);
                            mInitialisedTrackerNum++;
                        }
                        mInitialFlag = true;

                        BlackBoard::instance().updateSync(mKey, std::move(carTrackedRes));
                        sendAll(car_detect_available_atom_v, mKey);
                    } else {
                        VectorRect trackRectRes;
                        for(int i = 0; i < mInitialisedTrackerNum; ++i) {
                            const auto ok = mTrackers[i]->update(carTrackedRes.frame.frame, mTrackedBoxes[i]);
                            if(ok) {
                                trackRectRes.push_back(mTrackedBoxes[i]);
                            }
                        }

                        const auto computeCenterPoint = [](const cv::Rect& rect) {
                            return std::make_pair(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
                        };

                        //match algorithm: Match the rectangle closest to the center
                        for(const auto& detectedRect : carDetectedRes.cars) {
                            carTrackedRes.cars.push_back(detectedRect);  
                            const auto centerPoint = computeCenterPoint(detectedRect);
                            auto tmpDist = infinity;
                            auto matchIndex = -1;
                            for(int j = 0; j < trackRectRes.size(); ++j) {
                                const auto trackedRectCenter = computeCenterPoint(trackRectRes[j]);
                                const auto dist = (centerPoint.first - trackedRectCenter.first) * (centerPoint.first - trackedRectCenter.first) +
                                                  (centerPoint.second - trackedRectCenter.second) * (centerPoint.first - trackedRectCenter.first);
                                if(dist < tmpDist) {
                                    tmpDist = dist;
                                    matchIndex = j;
                                }
                            }
                            //If the mini distance between carTrackRect and the mathced carDetectedRec, 
                            //I think of this carTrackedRect as the missed dectected car. 
                            if(tmpDist > carDetectedRes.cars[matchIndex].width * carDetectedRes.cars[matchIndex].width) {
                                carTrackedRes.cars.push_back(trackRectRes[matchIndex]);
                            }
                        }

                        BlackBoard::instance().updateSync(mKey, std::move(carTrackedRes));
                        sendAll(car_detect_available_atom_v, mKey);

                        //Update
                        if(carDetectedRes.cars.size() > trackRectRes.size()) {
                            mTrackers.clear();
                            mTrackedBoxes.clear();
                            mInitialisedTrackerNum = 0;
                            for(const auto& detectedRect : carDetectedRes.cars) {
                                const auto tracker = cv::TrackerKCF::create();
                                mTrackers.push_back(tracker);
                                mTrackedBoxes.push_back(detectedRect);
                                tracker->init(carTrackedRes.frame.frame, mTrackedBoxes[mInitialisedTrackerNum++]);
                            }
                        }

                    }

                },

 
		};
	}

};

HUB_REGISTER_CLASS(CarTracker);
