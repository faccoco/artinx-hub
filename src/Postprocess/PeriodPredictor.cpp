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

static constexpr double sameThetaThreshold = glm::radians<double>(0.5);
static constexpr double minIntervalThreshold = 0.1;    // s
static constexpr double maxPeriodThreshold = 2;        // s
static constexpr double maxPeriodStdThreshold = 0.05;  // s
static constexpr int maxErrorTimes = 3;
static constexpr Duration minSendInterval = 1s;

class PeriodPredictor final : public HubHelper<caf::event_based_actor, void, period_predict_success_atom> {
    Identifier mKey;

    constexpr static size_t mPeriodTimesLen = 20;

    double mTargetTheta;
    std::deque<double> mPeriodTimes;
    std::optional<TimePoint> mLastSameYawTime;
    std::optional<TimePoint> mLastTime[4];
    int mErrorTimes;
    TimePoint mLastSend;
    enum State : unsigned char {
        APeriod = 0,
        BPeriod,
        CPeriod,
        DPeriod,
    } mState;

    glm::dvec3 getArmorPos(const DetectedTarget& armor, const Transform<FrameOfRef::Gun, FrameOfRef::Robot, true>& tfGun2Robot) {
        return tfGun2Robot(Vector<UnitType::Distance, FrameOfRef::Gun>(armor.center.mVal)).mVal;
    }

    inline static void step(State& a) {
        a = State((a + 1) & 0x3);
    }

    void clear() {
        logInfo("PeriodPredictor: clear all");
        mState = APeriod;
        mLastSameYawTime = std::nullopt;
        mPeriodTimes.clear();
        for(auto& times : mLastTime)
            times = std::nullopt;
        mErrorTimes = 0;
    }

    bool correct(int idx) {
        static constexpr double sameIntervalThreshold = 0.05;  // s
        static double lastDelta;
        double nowDelta;
        bool success = true;
        {
            double tmp = mPeriodTimes.back();
            mPeriodTimes.pop_back();
            nowDelta = tmp - mPeriodTimes.back();
        }
        logInfo(fmt::format("PeriodPredictor: nowDelta {}", nowDelta));
        if(mErrorTimes != 0) {
            if(lastDelta != 0 && std::abs(lastDelta - nowDelta) <= sameIntervalThreshold) {
                auto avgDelta = doubleCastDuration((lastDelta + nowDelta) / 2);
                for(int i = 0; i < idx; i++) {
                    if(i == idx)
                        continue;
                    if(mLastTime[i].has_value())
                        mLastTime[i].value() += avgDelta;
                }
                lastDelta = 0;
                logInfo("PeriodPredictor: try correct");
            } else {
                success = false;
                logInfo("PeriodPredictor: correct failed");
            }
        } else {
            lastDelta = nowDelta;
        }
        mErrorTimes += 1;
        return success;
    }

    double getTheta(const glm::dvec3& point) {
        return std::atan2(point.z, point.x);
    }

    double getPitch(const glm::dvec3& point) {
        return glm::asin(point.y / glm::length(point));
    }

public:
    PeriodPredictor(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](set_period_target_atom, Identifier key, bool init) {
                ACTOR_PROTOCOL_CHECK(set_period_target_atom, TypedIdentifier<SelectedTarget>, bool);
                ACTOR_EXCEPTION_PROBE();

                auto data = BlackBoard::instance().get<SelectedTarget>(key);
                auto tfGun2Robot = data->tfRobot2Gun.invTransformObj();

                // init
                if(init) {
                    clear();
                    mTargetTheta = getTheta(tfGun2Robot(Vector<UnitType::Distance, FrameOfRef::Gun>(0, 0, -1)).mVal);
                    logInfo(fmt::format("PeriodPredictor: inited yaw {} degree", glm::degrees(mTargetTheta)));
                }

                PredictedPeriodTarget res;
                res.lastUpdate = data->lastUpdate;

                bool findSameTheta = false;
                for(const auto& target : data->targets) {
                    if(target.motion == ArmorMotion::Static)
                        continue;

                    auto posRefRobot = getArmorPos(target, tfGun2Robot);
                    double thetaDelta = std::abs(getTheta(posRefRobot) - mTargetTheta);
                    if(thetaDelta > glm::pi<double>())
                        thetaDelta = glm::two_pi<double>() - thetaDelta;

                    if(thetaDelta <= sameThetaThreshold) {
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
                if(!mLastSameYawTime.has_value()) {
                    mLastSameYawTime = data->lastUpdate;
                    mLastTime[0] = data->lastUpdate;
                    step(mState);
                    // logInfo("PeriodPredictor: find first");
                    sendAll(period_predict_success_atom_v,
                            BlackBoard::instance().updateSync<PredictedPeriodTarget>(Identifier{ mKey.val }, res));
                    return;
                }

                // exclude extreme duration
                double interval = durationCastDouble(data->lastUpdate - mLastSameYawTime.value());
                if(interval > maxPeriodThreshold) {
                    clear();
                    logInfo(fmt::format("PeriodPredictor: interval: {} too large", interval));
                }
                if(interval < minIntervalThreshold)
                    return;

                if(mLastTime[mState].has_value()) {
                    // logInfo("PeriodPredictor: same pitch");
                    // logInfo(fmt::format("PeriodPredictor: pitch: {} degree", glm::degrees(getPitch(posRefRobot.mVal))));
                    if(mPeriodTimes.size() > mPeriodTimesLen)
                        mPeriodTimes.pop_front();
                    mPeriodTimes.push_back(durationCastDouble(data->lastUpdate - mLastTime[mState].value()));
                    double periodAvg = avg(mPeriodTimes);
                    double periodStd = Std(mPeriodTimes, periodAvg);
                    logInfo(fmt::format("PeriodPredictor: {} this period = {:.3f} period avg = {} | Std = {}", char('A' + mState),
                                        mPeriodTimes.back(), periodAvg, periodStd));
                    if(periodStd > maxPeriodStdThreshold) {
                        if(mErrorTimes >= maxErrorTimes) {
                            logInfo(fmt::format("PeriodPredictor: {} period std too large and all clear", char('A' + mState)));
                            clear();
                            return;
                        } else {
                            logInfo(fmt::format("PeriodPredictor: {} period std too large and correct", char('A' + mState)));
                            if(!correct(mState)) {
                                clear();
                            }
                        }
                    } else {
                        mErrorTimes = 0;
                        mLastTime[mState] = data->lastUpdate;
                        res.period = periodAvg;
                        if(SynchronizedClock::instance().now() - mLastSend > minSendInterval) {
                            mLastSend = SynchronizedClock::instance().now();
                            sendAll(period_predict_success_atom_v,
                                    BlackBoard::instance().updateSync<PredictedPeriodTarget>(Identifier{ mKey.val }, res));
                            logInfo("PeriodPredictor: send");
                        }
                    }
                }
                mLastTime[mState] = data->lastUpdate;
                mLastSameYawTime = data->lastUpdate;
                step(mState);
            },
        };
    }
};

HUB_REGISTER_CLASS(PeriodPredictor);