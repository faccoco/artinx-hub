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

static constexpr double sameThetaThreshold = glm::radians<double>(0.5);
static constexpr double minIntervalThreshold = 0.1;    // s
static constexpr double maxPeriodThreshold = 5;        // s
static constexpr double maxPeriodStdThreshold = 0.05;  // s
static constexpr int maxErrorTimes = 3;
static constexpr int maxEachErrorTimes = 2;
static constexpr Duration minSendInterval = 1s;

class PeriodPredictor final : public HubHelper<caf::event_based_actor, void, period_predict_success_atom> {
    Identifier mKey;

    double mTargetTheta;
    std::vector<double> mPeriodTimes[4];
    std::optional<TimePoint> mLastSameYawTime;
    std::optional<TimePoint> mLastTime[4];
    int mErrorTimes, mEachErrorTimes[4];
    TimePoint mLastSend;
    enum State : unsigned char {
        APeriod = 0,
        BPeriod,
        CPeriod,
        DPeriod,
    } mState;

    inline static void step(State& a) {
        a = State((a + 1) & 0x3);
    }

    void clear() {
        logInfo("PeriodPredictor: clear all");
        mState = APeriod;
        mLastSameYawTime = std::nullopt;
        for(auto& times : mPeriodTimes)
            times.clear();
        for(auto& times : mLastTime)
            times = std::nullopt;
        mErrorTimes = 0;
        std::memset(mEachErrorTimes, 0, sizeof(mEachErrorTimes));
    }

    void clear(int idx) {
        logInfo(fmt::format("PeriodPredictor: clear {}", char('A' + idx)));
        mPeriodTimes[idx].clear();
        mLastTime[idx] = std::nullopt;
        mErrorTimes = 0;
        mEachErrorTimes[idx] = 0;
    }

    bool correct(int idx) {
        static constexpr double sameIntervalThreshold = 0.1;  // s
        static double lastDelta;
        double nowDelta;
        bool success = true;
        {
            double tmp = mPeriodTimes[idx].back();
            mPeriodTimes[idx].pop_back();
            nowDelta = tmp - mPeriodTimes[idx].back();
        }
        logInfo(fmt::format("PeriodPredictor: nowDelta {}", nowDelta));
        if(mErrorTimes != 0) {
            if(lastDelta != 0 && std::abs(lastDelta - nowDelta) <= sameIntervalThreshold) {
                auto avgDelta = doubleCastDuration((lastDelta + nowDelta) / 2);
                for(auto& lastTime : mLastTime)
                    if(lastTime.has_value())
                        lastTime.value() += avgDelta;
                lastDelta = 0;
                logInfo("PeriodPredictor: try correct");
            } else {
                success = false;
                logInfo("PeriodPredictor: correct failed");
            }
        }
        mErrorTimes += 1;
        mEachErrorTimes[idx] += 1;
        lastDelta = nowDelta;
        return success;
    }

    double getTheta(const glm::dvec3& point) {
        double theta = glm::acos(point.x / glm::sqrt(point.x * point.x + point.z * point.z));
        return point.z < 0 ? glm::two_pi<double>() - theta : theta;
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
                if(!(data->selected.has_value() && data->tfRobot2Gun.has_value()))
                    return;
                auto tfGun2Robot = data->tfRobot2Gun->invTransformObj();
                HubLogger::watch("armor type", magic_enum::enum_name(data->selected->type));

                PredictedPeriodTarget res;
                res.lastUpdate = data->lastUpdate;

                Vector<UnitType::Distance, FrameOfRef::Gun> posOfRefGun(data->selected->center.mVal);
                Vector<UnitType::Distance, FrameOfRef::Robot> posRefRobot = tfGun2Robot(posOfRefGun);
                res.position = posRefRobot;

                // init
                if(init) {
                    clear();
                    mTargetTheta = getTheta(tfGun2Robot(Vector<UnitType::Distance, FrameOfRef::Gun>(0, 0, -1)).mVal);
                    logInfo(fmt::format("PeriodPredictor: inited theta {} degree", glm::degrees(mTargetTheta)));
                    return;
                }

                double thetaDelta = std::abs(getTheta(posRefRobot.mVal) - mTargetTheta);
                if(thetaDelta > glm::pi<double>())
                    thetaDelta = glm::two_pi<double>() - thetaDelta;

                // check
                if(thetaDelta <= sameThetaThreshold) {
                    // logInfo("PeriodPredictor: same theta");
                    // logInfo(
                    //     fmt::format("PeriodPredictor: pos:{} {} {}", posRefRobot.mVal.x, posRefRobot.mVal.y,
                    //     posRefRobot.mVal.z));
                    // logInfo(fmt::format("PeriodPredictor: theta: {} degree", glm::degrees(getTheta(posRefRobot.mVal))));

                    // logInfo(fmt::format("PeriodPredictor: mState: {}", mState));

                    if(!mLastSameYawTime.has_value()) {
                        mLastSameYawTime = data->lastUpdate;
                        mLastTime[0] = data->lastUpdate;
                        step(mState);
                        // logInfo("PeriodPredictor: find first");
                        sendAll(period_predict_success_atom_v,
                                BlackBoard::instance().updateSync<PredictedPeriodTarget>(Identifier{ mKey.val }, res));
                        return;
                    }
                    double interval = durationCastDouble(data->lastUpdate - mLastSameYawTime.value());
                    if(interval > maxPeriodThreshold) {
                        clear();
                        logInfo(fmt::format("PeriodPredictor: interval: {} too large", interval));
                    }
                    if(interval < minIntervalThreshold)
                        return;

                    mLastSameYawTime = data->lastUpdate;
                    if(!mLastTime[mState].has_value()) {
                        mLastTime[mState] = data->lastUpdate;
                    } else {
                        // logInfo("PeriodPredictor: same pitch");
                        // logInfo(fmt::format("PeriodPredictor: pitch: {} degree", glm::degrees(getPitch(posRefRobot.mVal))));
                        mPeriodTimes[mState].push_back(durationCastDouble(data->lastUpdate - mLastTime[mState].value()));
                        double periodAvg = avg(mPeriodTimes[mState]);
                        double periodStd = Std(mPeriodTimes[mState], periodAvg);
                        logInfo(fmt::format("PeriodPredictor: {} this period = {:.3f} period avg = {} | Std = {}",
                                            char('A' + mState), mPeriodTimes[mState].back(), periodAvg, periodStd));
                        if(periodStd > maxPeriodStdThreshold) {
                            if(mEachErrorTimes[mState] >= maxEachErrorTimes) {
                                logInfo(
                                    fmt::format("PeriodPredictor: {} period std too large and one clear", char('A' + mState)));
                                clear(mState);
                            } else if(mErrorTimes >= maxErrorTimes) {
                                logInfo(
                                    fmt::format("PeriodPredictor: {} period std too large and all clear", char('A' + mState)));
                                clear();
                                //                                mLastTime[mState] = data->lastUpdate;
                            } else {
                                logInfo(fmt::format("PeriodPredictor: {} period std too large and correct", char('A' + mState)));
                                if(!correct(mState)) {
                                    clear();
                                    //                                    mLastTime[mState] = data->lastUpdate;
                                }
                                mLastTime[mState] = data->lastUpdate;
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
                    step(mState);
                }
            },
        };
    }
};

HUB_REGISTER_CLASS(PeriodPredictor);