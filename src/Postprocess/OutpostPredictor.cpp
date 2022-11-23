#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "ExceptionProbe.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"

constexpr double staticVelThreshold = 0.1;
constexpr double staticPosThreshold = 0.01;
constexpr Duration maxWaitingTime = 50ms;
constexpr double maxJumpTheta = glm::radians<double>(10);

struct ArmorPredictorSettings final {
    bool enablePredictor;
};

template <class Inspector>
bool inspect(Inspector& f, ArmorPredictorSettings& x) {
    return f.object(x).fields(f.field("enablePredictor", x.enablePredictor));
}

class OutpostPredictor final : public HubHelper<caf::event_based_actor, ArmorPredictorSettings, outpost_predict_success_atom> {
    Identifier mKey, mIMUKey;
    GroupMask mGroupMask;

    std::queue<std::pair<TimePoint, Vector<UnitType::Distance, FrameOfRef::Robot>>> lastTwoPosition;
    std::optional<Scalar<UnitType::Angle>> lastTheta;

    // a,b,c mustn't be on the same line or on the same point
    glm::dvec3 circleCenter(glm::dvec3 a, glm::dvec3 b, glm::dvec3 c) {
        double a1, b1, c1, d1;
        double a2, b2, c2, d2;
        double a3, b3, c3, d3;

        double la, lb, lc, l;

        glm::dvec3 res;

        la = a.x * a.x + a.y * a.y + a.z * a.z;
        lb = b.x * b.x + b.y * b.y + b.z * b.z;
        lc = c.x * c.x + c.y * c.y + c.z * c.z;

        a1 = (a.y * b.z - b.y * a.z - a.y * c.z + c.y * a.z + b.y * c.z - c.y * b.z);
        b1 = -(a.x * b.z - b.x * a.z - a.x * c.z + c.x * a.z + b.x * c.z - c.x * b.z);
        c1 = (a.x * b.y - b.x * a.y - a.x * c.y + c.x * a.y + b.x * c.y - c.x * b.y);
        d1 = a.x * b.z * c.y + a.y * b.x * c.z + a.z * b.y * c.x - a.x * b.y * c.z - a.y * b.z * c.x - a.z * b.x * c.y;

        a2 = 2 * (b.x - a.x);
        b2 = 2 * (b.y - a.y);
        c2 = 2 * (b.z - a.z);
        d2 = la - lb;

        a3 = 2 * (c.x - a.x);
        b3 = 2 * (c.y - a.y);
        c3 = 2 * (c.z - a.z);
        d3 = la - lc;

        l = a1 * b2 * c3 + a2 * b3 * c1 + a3 * b1 * c2 - a1 * b3 * c2 - a2 * b1 * c3 - a3 * b2 * c1;

        res.x = -(b1 * c2 * d3 + b2 * c3 * d1 + b3 * c1 * d2 - b1 * c3 * d2 - b2 * c1 * d3 - b3 * c2 * d1) / l;
        res.y = (a1 * c2 * d3 + a2 * c3 * d1 + a3 * c1 * d2 - a1 * c3 * d2 - a2 * c1 * d3 - a3 * c2 * d1) / l;
        res.z = -(a1 * b2 * d3 + a2 * b3 * d1 + a3 * b1 * d2 - a1 * b3 * d2 - a2 * b1 * d3 - a3 * b2 * d1) / l;

        return res;
    }

public:
    OutpostPredictor(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](set_outpost_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(set_outpost_atom, TypedIdentifier<SelectedTarget>);
                ACTOR_EXCEPTION_PROBE();

                auto data = BlackBoard::instance().get<SelectedTarget>(key);
                const auto dataPosture = BlackBoard::instance().get<PostureData>(mIMUKey);
                if(!(data->selected.has_value() && dataPosture.has_value() && data->tfRobot2Gun.has_value()))
                    return;
                if(!(std::fabs(dataPosture->linearVelocityOfRobot.mVal.x) <= staticVelThreshold &&
                     std::fabs(dataPosture->linearVelocityOfRobot.mVal.y) <= staticVelThreshold &&
                     std::fabs(dataPosture->linearVelocityOfRobot.mVal.z) <= staticVelThreshold)) {
                    logInfo("OutpostPredictor: out of static vel threshold");
                    lastTwoPosition.pop();
                    lastTwoPosition.pop();
                    lastTheta = std::nullopt;
                    return;
                }
                HubLogger::watch("armor type", magic_enum::enum_name(data->selected->type));

                PredictedOutpost res;
                res.lastUpdate = data->lastUpdate;

                Vector<UnitType::Distance, FrameOfRef::Gun> posOfRefGun(data->selected->center.mVal);
                Vector<UnitType::Distance, FrameOfRef::Robot> posRefRobot = data->tfRobot2Gun->invTransform(posOfRefGun);

                if(mConfig.enablePredictor) {  // 如果使用预测功能的话，修正旋转中心及当前位置
                } else {                       // 如果不使用预测功能的话，不修正位置
                }
                {  //下面是不使用预测的
                    static int stopTimes = 0;
                    static bool stopPrinted = false;
                    static std::optional<int> direction;
                    if(stopTimes > 5) {
                        if(!stopPrinted) {
                            logInfo("stop");
                            stopPrinted = true;
                        }
                        res.angularVelocity = 0;
                        res.theta = glm::radians(90.0);
                        res.centerOfOutpost = posRefRobot;
                        res.centerOfOutpost.mVal.z -= radiusOfOutpost;
                    } else {
                        if(lastTwoPosition.size() == 0) {
                            lastTwoPosition.push(std::make_pair(data->lastUpdate, posRefRobot));
                            return;
                        }
                        if(posRefRobot == lastTwoPosition.back().second || posRefRobot == lastTwoPosition.front().second) {
                            stopTimes += 1;
                            return;
                        }
                        if(lastTwoPosition.size() == 1) {
                            lastTwoPosition.push(std::make_pair(data->lastUpdate, posRefRobot));
                            return;
                        }
                        if(data->lastUpdate - lastTwoPosition.front().first > maxWaitingTime) {
                            if(glm::distance(posRefRobot.mVal, lastTwoPosition.front().second.mVal) > staticPosThreshold) {
                                logInfo("remove stop");
                                stopPrinted = false;
                                stopTimes = 0;
                            } else {
                                stopTimes += 1;
                            }
                            lastTwoPosition.pop();
                            lastTwoPosition.push(std::make_pair(data->lastUpdate, posRefRobot));
                            lastTheta = std::nullopt;
                            return;
                        }
                        res.centerOfOutpost = circleCenter(lastTwoPosition.front().second.mVal,
                                                           lastTwoPosition.back().second.mVal, posRefRobot.mVal);
                        {
                            double x = posRefRobot.mVal.x - res.centerOfOutpost.mVal.x;
                            double z = posRefRobot.mVal.z - res.centerOfOutpost.mVal.z;
                            double l = glm::sqrt(x * x + z * z);
                            res.theta = glm::acos(x / l);
                        }
                        if(!lastTheta.has_value()) {
                            double x = lastTwoPosition.back().second.mVal.x - res.centerOfOutpost.mVal.x;
                            double z = lastTwoPosition.back().second.mVal.z - res.centerOfOutpost.mVal.z;
                            double l = glm::sqrt(x * x + z * z);
                            lastTheta = glm::acos(x / l);
                        }
                        if(!direction.has_value()) {
                            if(std::fabs((lastTheta.value() - res.theta).mVal) > maxJumpTheta)
                                direction = (lastTheta.value() > res.theta ? 1 : -1);
                            else
                                direction = (res.theta > lastTheta.value() ? 1 : -1);
                        }
                        if(std::fabs((lastTheta.value() - res.theta).mVal) > maxJumpTheta)
                            lastTheta.value().mVal -= direction.value() * glm::radians<double>(120);
                        res.angularVelocity = (res.theta - lastTheta.value()) /
                            Scalar<UnitType::Time>{ static_cast<double>(
                                                        (data->lastUpdate - lastTwoPosition.back().first).count()) /
                                                    Clock::period::den * Clock::period::num };
                        lastTheta = res.theta;
                        lastTwoPosition.pop();
                        lastTwoPosition.push(std::make_pair(data->lastUpdate, posRefRobot));
                    }
                }

                sendAll(outpost_predict_success_atom_v,
                        BlackBoard::instance().updateSync<PredictedOutpost>(Identifier{ mKey.val }, res));
            },
            [this](update_posture_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(update_posture_atom, TypedIdentifier<PostureData>);
                mIMUKey = key;
            },
        };
    }
};

HUB_REGISTER_CLASS(OutpostPredictor);
