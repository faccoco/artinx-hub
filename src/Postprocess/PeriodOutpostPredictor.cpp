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
static constexpr double minPeriodThreshold = 0.8;     // s
static constexpr double maxPeriodThreshold = 10;    // s
static constexpr double maxPeriodStdThreshold = 0.2;  // s

class PeriodOutpostPredictor final : public HubHelper<caf::event_based_actor, void, period_predict_success_atom> {
    Identifier mKey;

    double mTargetTheta;
    double mTargetPitch;
    std::vector<double> mPeriodTimes;
    std::optional<TimePoint> mLastTime;

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
        : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](set_period_outpost_atom, Identifier key, bool init) {
                ACTOR_PROTOCOL_CHECK(set_period_outpost_atom, TypedIdentifier<SelectedTarget>, bool);
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
                    mTargetTheta = getTheta(posRefRobot.mVal);
                    logInfo("PeriodOutpostPredictor: inited");
                    return;
                }

                if(!mLastTime.has_value()) {
                    mTargetPitch = getPitch(posRefRobot.mVal);
                    mLastTime = data->lastUpdate;
                    return;
                }

                double timeGap = durationCastDouble(data->lastUpdate - mLastTime.value());

                HubLogger::watch("timeGap",timeGap);

                if(timeGap > maxPeriodThreshold) {
                    clear();
                    logInfo(fmt::format("PeriodOutpostPredictor: timeGap: {} too large",timeGap));
                }
                if(timeGap < minPeriodThreshold)
                    return;

                double thetaDelta=std::abs(getTheta(posRefRobot.mVal) - mTargetTheta);

                HubLogger::watch("thetaDelta",thetaDelta);
                // check
                if(thetaDelta <= sameThetaThreshold) {
                    logInfo("PeriodOutpostPredictor: same theta");
                    logInfo(fmt::format("PeriodOutpostPredictor: pos:{} {} {}", posRefRobot.mVal.x, posRefRobot.mVal.y,
                                        posRefRobot.mVal.z));
                    logInfo(fmt::format("PeriodOutpostPredictor: theta: {} degree", glm::degrees(getTheta(posRefRobot.mVal))));
                    if(double pitchDelta=std::abs(getPitch(posRefRobot.mVal) - mTargetPitch);pitchDelta > samePitchThreshold) {
                        clear();
                        logInfo(fmt::format("PeriodOutpostPredictor: pitchDelta: {} too large",pitchDelta));
                        return;
                    }
                    logInfo("PeriodOutpostPredictor: same pitch");
                    logInfo(fmt::format("PeriodOutpostPredictor: pitch: {} degree", glm::degrees(getPitch(posRefRobot.mVal))));
                    mPeriodTimes.push_back(timeGap);
                    mLastTime = data->lastUpdate;
                    double periodAvg = avg(mPeriodTimes);
                    double periodStd = Std(mPeriodTimes, periodAvg);
                    logInfo(fmt::format("PeriodOutpostPredictor: avg = {} | Std = {}", periodAvg, periodStd));
                    if(periodStd > maxPeriodStdThreshold) {
                        clear();
                        return;
                    }
                    res.period = periodAvg;
                    sendAll(period_predict_success_atom_v,
                            BlackBoard::instance().updateSync<PredictedPeriodTarget>(Identifier{ mKey.val }, res));
                }
            },
        };
    }
};

HUB_REGISTER_CLASS(PeriodOutpostPredictor);