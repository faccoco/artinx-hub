#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "ExceptionProbe.hpp"
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
};

template <class Inspector>
bool inspect(Inspector& f, PeriodOutpostPredictorSettings& x) {
    return f.object(x).fields(f.field("sameThetaThreshold", x.sameThetaThreshold).fallback(0.1),
                              f.field("samePitchThreshold", x.samePitchThreshold).fallback(1),
                              f.field("minIntervalThreshold", x.minIntervalThreshold).fallback(0.5),
                              f.field("maxPeriodThreshold", x.maxPeriodThreshold).fallback(2),
                              f.field("maxPeriodStdThreshold", x.maxPeriodStdThreshold).fallback(0.015));
}

class PeriodOutpostPredictor final
    : public HubHelper<caf::event_based_actor, PeriodOutpostPredictorSettings, period_predict_success_atom> {
    Identifier mKey;

    constexpr static size_t mPeriodTimesLen = 10;

    const double mSameThetaThreshold;
    const double mSamePitchThreshold;

    double mTargetTheta;
    double mTargetPitch;
    std::deque<double> mPeriodTimes;
    std::optional<TimePoint> mLastTime;

    glm::dvec3 getArmorPos(const DetectedTarget& armor, const Transform<FrameOfRef::Gun, FrameOfRef::Robot, true>& tfGun2Robot) {
        return tfGun2Robot(Vector<UnitType::Distance, FrameOfRef::Gun>(armor.center.mVal)).mVal;
    }

    void clear() {
        logInfo("PeriodOutpostPredictor: clear");
        mPeriodTimes.clear();
        mLastTime = std::nullopt;
    }

    double getTheta(const glm::dvec3& point) {
        return std::atan2(point.z, point.x);
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
                auto tfGun2Robot = data->tfRobot2Gun.invTransformObj();

                // init
                if(init) {
                    mLastTime = std::nullopt;
                    mTargetTheta = getTheta(tfGun2Robot(Vector<UnitType::Distance, FrameOfRef::Gun>(0, 0, -1)).mVal);
                    logInfo(fmt::format("PeriodOutpostPredictor: inited yaw {} degree", glm::degrees(mTargetTheta)));
                }

                PredictedPeriodTarget res;
                res.lastUpdate = data->lastUpdate;

                // find same theta armor
                bool findSameTheta = false;
                for(const auto& target : data->targets) {
                    if(target.motion == ArmorMotion::Static ||
                       (target.id == RobotType::Negative && target.type == ArmorType::Large))
                        continue;

                    auto posRefRobot = getArmorPos(target, tfGun2Robot);
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
                    mTargetPitch = getPitch(res.position->mVal);
                    mLastTime = data->lastUpdate;
                    //                    logInfo("PeriodOutpostPredictor: find first");
                    // use last period if have
                    if(!mPeriodTimes.empty()) {
                        double periodAvg = avg(mPeriodTimes);
                        double periodStd = Std(mPeriodTimes, periodAvg);
                        logInfo(fmt::format("PeriodOutpostPredictor: avg = {} | Std = {}", periodAvg, periodStd));
                        HubLogger::VisualLog(fmt::format("PeriodOutpostPredictor: avg = {} | Std = {}", periodAvg, periodStd));
                        if(periodStd > mConfig.maxPeriodStdThreshold) {
                            clear();
                            return;
                        }
                        res.period = periodAvg;
                    }
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
                if(double pitchDelta = std::abs(getPitch(res.position->mVal) - mTargetPitch); pitchDelta > mSamePitchThreshold) {
                    clear();
                    logInfo(fmt::format("PeriodOutpostPredictor: pitchDelta: {} too large", pitchDelta));
                    return;
                }
                // logInfo("PeriodOutpostPredictor: same pitch");
                // logInfo(fmt::format("PeriodOutpostPredictor: pitch: {} degree", glm::degrees(getPitch(posRefRobot.mVal))));

                // calculate period
                mLastTime = data->lastUpdate;
                if(mPeriodTimes.size() > mPeriodTimesLen)
                    mPeriodTimes.pop_front();
                mPeriodTimes.push_back(interval);
                double periodAvg = avg(mPeriodTimes);
                double periodStd = Std(mPeriodTimes, periodAvg);
                logInfo(fmt::format("PeriodOutpostPredictor: avg = {} | Std = {}", periodAvg, periodStd));
                HubLogger::VisualLog(fmt::format("PeriodOutpostPredictor: avg = {} | Std = {}", periodAvg, periodStd));
                if(periodStd > mConfig.maxPeriodStdThreshold) {
                    clear();
                    return;
                }
                res.period = periodAvg;
                if(mPeriodTimes.size() > 1)
                    res.position = std::nullopt;
                sendAll(period_predict_success_atom_v,
                        BlackBoard::instance().updateSync<PredictedPeriodTarget>(Identifier{ mKey.val }, res));
            },
        };
    }
};

HUB_REGISTER_CLASS(PeriodOutpostPredictor);