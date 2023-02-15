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
static constexpr double samePitchThreshold = glm::radians<double>(0.5);
static constexpr double minPeriodThreshold = 0.2;     // s
static constexpr double maxPeriodThreshold = 3.5;     // s
static constexpr double maxPeriodStdThreshold = 0.2;  // s

class PeriodPredictor final : public HubHelper<caf::event_based_actor, void, period_predict_success_atom> {
    Identifier mKey;

    double mTargetTheta;
    double mTargetPitch[4];
    std::vector<double> mPeriodTimes[4];
    TimePoint mLastTime[4];
    enum State : unsigned char {
        firstAPeriod = 0,
        firstBPeriod,
        firstCPeriod,
        firstDPeriod,
        APeriod,
        BPeriod,
        CPeriod,
        DPeriod,
    } mState;

    inline static bool isNormalPeriod(State a) {
        return a & 0x4;
    }

    inline static bool isFirstPeriod(State a) {
        return !isNormalPeriod(a);
    }

    inline static int getIdx(State a) {
        return a & 0x3;
    }

    inline static void step(State& a) {
        a = (a == DPeriod ? APeriod : State(a + 1));
    }

    void clear() {
        logInfo("PeriodPredictor: clear");
        mState = firstAPeriod;
        for(auto& times : mPeriodTimes)
            times.clear();
    }

    double getTheta(const glm::dvec3& point) {
        double theta = glm::acos(point.x / glm::sqrt(point.x * point.x + point.z * point.z));
        return point.z < 0 ? glm::two_pi<double>() - theta : theta;
    }

    double getPitch(const glm::dvec3& point) {
        return glm::asin(point.y / glm::length(point));
    }

public:
    PeriodPredictor(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ generateKey(this) }, mState(firstAPeriod) {}
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
                    logInfo("PeriodPredictor: inited");
                    return;
                }

                // check
                if(std::abs(getTheta(posRefRobot.mVal) - mTargetTheta) <= sameThetaThreshold) {
                    // logInfo("PeriodPredictor: same theta");
                    // logInfo(
                    //     fmt::format("PeriodPredictor: pos:{} {} {}", posRefRobot.mVal.x, posRefRobot.mVal.y, posRefRobot.mVal.z));
                    // logInfo(fmt::format("PeriodPredictor: theta: {} degree", glm::degrees(getTheta(posRefRobot.mVal))));

                    // logInfo(fmt::format("PeriodPredictor: mState: {}", mState));

                    if(mState == firstAPeriod) {
                        mTargetPitch[0] = getPitch(posRefRobot.mVal);
                        mLastTime[0] = data->lastUpdate;
                        step(mState);
                        // logInfo("PeriodPredictor: find first");
                        return;
                    }
                    double timeGap = durationCastDouble(data->lastUpdate - mLastTime[(mState - 1) & 0x3]);
                    if(timeGap > maxPeriodThreshold) {
                        clear();
                        logInfo(fmt::format("PeriodPredictor: timeGap: {} too large", timeGap));
                    }
                    if(timeGap < minPeriodThreshold)
                        return;

                    int idx = getIdx(mState);
                    if(isFirstPeriod(mState)) {
                        mTargetPitch[idx] = getPitch(posRefRobot.mVal);
                        mLastTime[idx] = data->lastUpdate;
                    } else {
                        if(std::abs(getPitch(posRefRobot.mVal) - mTargetPitch[idx]) > samePitchThreshold) {
                            clear();
                            return;
                        }
                        // logInfo("PeriodPredictor: same pitch");
                        // logInfo(fmt::format("PeriodPredictor: pitch: {} degree", glm::degrees(getPitch(posRefRobot.mVal))));
                        mPeriodTimes[idx].push_back(durationCastDouble(data->lastUpdate - mLastTime[idx]));
                        mLastTime[idx] = data->lastUpdate;
                        double periodAvg = avg(mPeriodTimes[idx]);
                        double periodStd = Std(mPeriodTimes[idx], periodAvg);
                        logInfo(
                            fmt::format("PeriodPredictor: {} period avg = {} | Std = {}", char('A' + idx), periodAvg, periodStd));
                        if(periodStd > maxPeriodStdThreshold) {
                            clear();
                            return;
                        }
                        res.period = periodAvg;
                        sendAll(period_predict_success_atom_v,
                                BlackBoard::instance().updateSync<PredictedPeriodTarget>(Identifier{ mKey.val }, res));
                    }
                    step(mState);
                }
            },
        };
    }
};

HUB_REGISTER_CLASS(PeriodPredictor);