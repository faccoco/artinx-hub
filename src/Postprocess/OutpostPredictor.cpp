#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "ExceptionProbe.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
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
    Identifier mKey;
    GroupMask mGroupMask;

    std::queue<std::pair<TimePoint, Vector<UnitType::Distance, FrameOfRef::Robot>>> mLastTwoPosition;
    std::optional<Scalar<UnitType::Angle>> mLastTheta;
    int stopTimes = 0;

    // a,b,c mustn't be on the same line or on the same point
    glm::dvec3 circleCenter(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c) {
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
                if(!(data->selected.has_value() && data->tfRobot2Gun.has_value()))
                    return;
                HubLogger::watch("armor type", magic_enum::enum_name(data->selected->type));

                PredictedOutpost res;
                res.lastUpdate = data->lastUpdate;

                Vector<UnitType::Distance, FrameOfRef::Gun> posOfRefGun(data->selected->center.mVal);
                Vector<UnitType::Distance, FrameOfRef::Robot> posRefRobot = data->tfRobot2Gun->invTransform(posOfRefGun);

                if(mConfig.enablePredictor) {  // 如果使用预测功能的话，修正旋转中心及当前位置
                } else {                       // 如果不使用预测功能的话，不修正位置
                }
                {  //下面是不使用预测的

                    static std::optional<int> direction;
                    if(stopTimes > 5) {
                        logInfo("outpost stop");
                        res.angularVelocity = 0;
                        res.theta = glm::radians(90.0);
                        res.centerOfOutpost = posRefRobot;
                        res.centerOfOutpost.mVal.z -= radiusOfOutpost;
                    } else {
                        if(mLastTwoPosition.size() == 0) {
                            mLastTwoPosition.push(std::make_pair(data->lastUpdate, posRefRobot));
                            return;
                        }
                        if(posRefRobot == mLastTwoPosition.back().second || posRefRobot == mLastTwoPosition.front().second) {
                            stopTimes += 1;
                            return;
                        }
                        if(mLastTwoPosition.size() == 1) {
                            mLastTwoPosition.push(std::make_pair(data->lastUpdate, posRefRobot));
                            return;
                        }
                        if(data->lastUpdate - mLastTwoPosition.front().first > maxWaitingTime) {
                            if(glm::distance(posRefRobot.mVal, mLastTwoPosition.front().second.mVal) > staticPosThreshold) {
                                logInfo("outpost remove stop");
                                stopTimes = 0;
                            } else {
                                stopTimes += 1;
                            }
                            mLastTwoPosition.pop();
                            mLastTwoPosition.push(std::make_pair(data->lastUpdate, posRefRobot));
                            mLastTheta = std::nullopt;
                            return;
                        }
                        res.centerOfOutpost = circleCenter(mLastTwoPosition.front().second.mVal,
                                                           mLastTwoPosition.back().second.mVal, posRefRobot.mVal);
                        {
                            double x = posRefRobot.mVal.x - res.centerOfOutpost.mVal.x;
                            double z = posRefRobot.mVal.z - res.centerOfOutpost.mVal.z;
                            double l = glm::sqrt(x * x + z * z);
                            res.theta = glm::acos(x / l);
                        }
                        if(!mLastTheta.has_value()) {
                            double x = mLastTwoPosition.back().second.mVal.x - res.centerOfOutpost.mVal.x;
                            double z = mLastTwoPosition.back().second.mVal.z - res.centerOfOutpost.mVal.z;
                            double l = glm::sqrt(x * x + z * z);
                            mLastTheta = glm::acos(x / l);
                        }
                        if(!direction.has_value()) {
                            if(std::fabs((mLastTheta.value() - res.theta).mVal) > maxJumpTheta)
                                direction = (mLastTheta.value() > res.theta ? 1 : -1);
                            else
                                direction = (res.theta > mLastTheta.value() ? 1 : -1);
                        }
                        if(std::fabs((mLastTheta.value() - res.theta).mVal) > maxJumpTheta)
                            mLastTheta.value().mVal -= direction.value() * glm::radians<double>(120);
                        res.angularVelocity = (res.theta - mLastTheta.value()) /
                            Scalar<UnitType::Time>{ static_cast<double>(
                                                        (data->lastUpdate - mLastTwoPosition.back().first).count()) /
                                                    Clock::period::den * Clock::period::num };
                        mLastTheta = res.theta;
                        mLastTwoPosition.pop();
                        mLastTwoPosition.push(std::make_pair(data->lastUpdate, posRefRobot));
                    }
                }

                sendAll(outpost_predict_success_atom_v,
                        BlackBoard::instance().updateSync<PredictedOutpost>(Identifier{ mKey.val }, res));
            },
        };
    }
};

HUB_REGISTER_CLASS(OutpostPredictor);
