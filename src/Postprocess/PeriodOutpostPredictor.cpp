#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "ExceptionProbe.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include "Timer.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <magic_enum.hpp>
#include <optional>

#include "SuppressWarningEnd.hpp"

struct PeriodOutpostPredictorSettings final {
    double sameThetaThreshold;
    double samePitchThreshold;
    double minIntervalThreshold;   // s
    double maxPeriodThreshold;     // s
    double maxPeriodStdThreshold;  // s
    double staticImgPosThreshold;
    double maxMatchImgDistance;
};

template <class Inspector>
bool inspect(Inspector& f, PeriodOutpostPredictorSettings& x) {
    return f.object(x).fields(f.field("sameThetaThreshold", x.sameThetaThreshold).fallback(0.1),
                              f.field("samePitchThreshold", x.samePitchThreshold).fallback(1),
                              f.field("minIntervalThreshold", x.minIntervalThreshold).fallback(0.1),
                              f.field("maxPeriodThreshold", x.maxPeriodThreshold).fallback(5),
                              f.field("maxPeriodStdThreshold", x.maxPeriodStdThreshold).fallback(0.015),
                              f.field("staticImgPosThreshold", x.staticImgPosThreshold).fallback(10),
                              f.field("maxMatchImgDistance", x.maxMatchImgDistance).fallback(10));
}

class PeriodOutpostPredictor final
    : public HubHelper<caf::event_based_actor, PeriodOutpostPredictorSettings, period_predict_success_atom> {
    Identifier mKey;

    constexpr static Duration mTrackValidTime = 50ms;
    constexpr static int mTrackSize = 10;

    const double mSameThetaThreshold;
    const double mSamePitchThreshold;

    double mTargetTheta;
    double mTargetPitch;
    std::vector<double> mPeriodTimes;
    std::optional<TimePoint> mLastTime;
    Transform<FrameOfRef::Gun, FrameOfRef::Robot, true> mTfGun2Robot;
    std::list<std::queue<std::pair<TimePoint, cv::Point2f>>> mTrackedArmors;

    glm::dvec3 getArmorPos(const DetectedTarget& armor) {
        return mTfGun2Robot(Vector<UnitType::Distance, FrameOfRef::Gun>(armor.center.mVal)).mVal;
    }

    void clear() {
        logInfo("PeriodOutpostPredictor: clear");
        mPeriodTimes.clear();
        mLastTime = std::nullopt;
    }

    double getTheta(const glm::dvec3& point) {
        double theta = glm::acos(point.x / glm::sqrt(point.x * point.x + point.z * point.z));
        return point.z < 0 ? glm::two_pi<double>() - theta : theta;
    }

    double getPitch(const glm::dvec3& point) {
        return glm::asin(point.y / glm::length(point));
    }

public:
    PeriodOutpostPredictor(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ generateKey(this) }, mSameThetaThreshold(glm::radians(mConfig.sameThetaThreshold)),
          mSamePitchThreshold(glm::radians(mConfig.samePitchThreshold)) {}
    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](set_period_outpost_atom, Identifier key, bool init) {
                ACTOR_PROTOCOL_CHECK(set_period_outpost_atom, TypedIdentifier<SelectedTarget>, bool);
                ACTOR_EXCEPTION_PROBE();

                auto data = BlackBoard::instance().get<SelectedTarget>(key);
                mTfGun2Robot = data->tfRobot2Gun.invTransformObj();

                // init
                if(init) {
                    clear();
                    mTargetTheta = getTheta(mTfGun2Robot(Vector<UnitType::Distance, FrameOfRef::Gun>(0, 0, -1)).mVal);
                    logInfo(fmt::format("PeriodOutpostPredictor: inited yaw {} degree", glm::degrees(mTargetTheta)));
                    return;
                }

                if(data->targets.empty())
                    return;

                PredictedPeriodTarget res;
                res.lastUpdate = data->lastUpdate;

                // exclude invalid data in mTrackedArmors
                for(auto trackedIter = mTrackedArmors.begin(); trackedIter != mTrackedArmors.end(); trackedIter++) {
                    while(!trackedIter->empty() && trackedIter->front().first - res.lastUpdate > mTrackValidTime)
                        trackedIter->pop();
                    if(trackedIter->empty())
                        mTrackedArmors.erase(trackedIter);
                }

                bool findSameTheta = false;
                for(const auto& target : data->targets) {
                    // process whether armor is static
                    std::optional<bool> isStatic;
                    for(auto& tracked : mTrackedArmors) {
                        // matched
                        if(distance2D(tracked.back().second, target.armorImgCenter) < mConfig.maxMatchImgDistance) {
                            isStatic = distance2D(tracked.front().second, target.armorImgCenter) < mConfig.staticImgPosThreshold;
                            if(tracked.size() == mTrackSize)
                                tracked.pop();
                            tracked.emplace(data->lastUpdate, target.armorImgCenter);
                            break;
                        }
                    }
                    // not find match armor
                    if(!isStatic.has_value()) {
                        mTrackedArmors.emplace_back();
                        mTrackedArmors.back().emplace(data->lastUpdate, target.armorImgCenter);
                        continue;
                    }
                    // exclude static armor
                    if(isStatic)
                        continue;

                    auto posRefRobot = getArmorPos(target);
                    double thetaDelta = std::abs(getTheta(posRefRobot) - mTargetTheta);
                    if(thetaDelta > glm::pi<double>())
                        thetaDelta = glm::two_pi<double>() - thetaDelta;

                    if(thetaDelta <= mSameThetaThreshold) {
                        res.position = posRefRobot;
                        findSameTheta = true;
                        // logInfo("PeriodOutpostPredictor: same yaw");
                        // logInfo(fmt::format("PeriodOutpostPredictor: pos:{} {} {}", posRefRobot.mVal.x, posRefRobot.mVal.y,
                        //                     posRefRobot.mVal.z));
                        // logInfo(fmt::format("PeriodOutpostPredictor: yaw: {} degree",
                        // glm::degrees(getTheta(posRefRobot.mVal))));
                        break;
                    }
                }
                if(!findSameTheta)
                    return;

                // find first target, init pitch and lastTime
                if(!mLastTime.has_value()) {
                    mTargetPitch = getPitch(res.position.mVal);
                    mLastTime = data->lastUpdate;
                    // logInfo("PeriodOutpostPredictor: find first");
                    sendAll(period_predict_success_atom_v,
                            BlackBoard::instance().updateSync<PredictedPeriodTarget>(Identifier{ mKey.val }, res));
                    return;
                }

                // exclude extreme duration
                double interval = durationCastDouble(data->lastUpdate - mLastTime.value());
                if(interval > mConfig.maxPeriodThreshold) {
                    clear();
                    logInfo(fmt::format("PeriodOutpostPredictor: interval: {} too large", interval));
                }
                if(interval < mConfig.minIntervalThreshold)
                    return;

                // judge same pitch
                if(double pitchDelta = std::abs(getPitch(res.position.mVal) - mTargetPitch); pitchDelta > mSamePitchThreshold) {
                    clear();
                    logInfo(fmt::format("PeriodOutpostPredictor: pitchDelta: {} too large", pitchDelta));
                    return;
                }
                // logInfo("PeriodOutpostPredictor: same pitch");
                // logInfo(fmt::format("PeriodOutpostPredictor: pitch: {} degree", glm::degrees(getPitch(posRefRobot.mVal))));

                // calculate period
                mPeriodTimes.push_back(interval);
                mLastTime = data->lastUpdate;
                double periodAvg = avg(mPeriodTimes);
                double periodStd = Std(mPeriodTimes, periodAvg);
                logInfo(fmt::format("PeriodOutpostPredictor: avg = {} | Std = {}", periodAvg, periodStd));
                if(periodStd > mConfig.maxPeriodStdThreshold) {
                    clear();
                    return;
                }
                res.period = periodAvg;
                sendAll(period_predict_success_atom_v,
                        BlackBoard::instance().updateSync<PredictedPeriodTarget>(Identifier{ mKey.val }, res));
            },
        };
    }
};

HUB_REGISTER_CLASS(PeriodOutpostPredictor);