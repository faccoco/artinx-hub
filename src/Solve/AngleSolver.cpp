#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"
#include "Utility.hpp"
#include <caf/event_based_actor.hpp>
#include <cmath>
#include <cstdint>
#include <fmt/format.h>
struct AngleSolverSettings final {
    double precision;
};

template <class Inspector>
bool inspect(Inspector& f, AngleSolverSettings& x) {
    return f.object(x).fields(f.field("precision", x.precision));
}

class AngleSolver final : public HubHelper<caf::event_based_actor, AngleSolverSettings, set_target_info_atom> {
    Identifier mKey, mIMUKey, mHeadKey;

public:
    AngleSolver(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(AngleSolver).hash_code() } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [this](set_target_atom, Identifier key) {
                     const auto data = BlackBoard::instance().get<SelectedTarget>(key);
                     const auto dataHeadinfo = BlackBoard::instance().get<HeadInfo>(mHeadKey);
                     const auto dataPosture = BlackBoard::instance().get<PostureData>(mIMUKey);
                     const auto globalSettings = BlackBoard::instance().get<GlobalSettings>({}).value();
                     const double g = -globalSettings.gForce, bulletSpeed = 25.00;
                     auto square = [=](double x) { return x * x; };
                     auto cube = [=](double x) { return x * x * x; };

                     Vector<UnitType::Distance, FrameOfReference::Gun> positionOfReferenceGun(data.value().center.value().raw());
                     glm::dvec3 transformedPosition = { 0, 0, 0 };
                     glm::dvec3 forwardVector = { 0, 0, -1 };
                     Vector<UnitType::Distance, FrameOfReference::Gun> forwardPosition(forwardVector);
                     if((dataHeadinfo.has_value()) && (dataPosture.has_value())) {
                         Vector<UnitType::Distance, FrameOfReference::Robot> positionOfReferenceRobot =
                             dataHeadinfo.value().transform(positionOfReferenceGun);
                         Vector<UnitType::Distance, FrameOfReference::Ground> positionOfReferenceGround =
                             dataPosture.value().postureOfRobot(positionOfReferenceRobot);
                         Vector<UnitType::Distance, FrameOfReference::Robot> forwardPositionOfReferenceRobot =
                             dataHeadinfo.value().transform(forwardPosition);
                         Vector<UnitType::Distance, FrameOfReference::Ground> forwardPositionOfReferenceGround =
                             dataPosture.value().postureOfRobot(forwardPositionOfReferenceRobot);
                         transformedPosition = { positionOfReferenceGround.raw().x, -positionOfReferenceGround.raw().z,
                                                 positionOfReferenceGround.raw().y };
                         forwardVector = { forwardPositionOfReferenceGround.raw().x, -forwardPositionOfReferenceGround.raw().z,
                                           forwardPositionOfReferenceGround.raw().y };
                         //(forward:+y,right:+x)
                         // logInfo(fmt::format("transformedposition: {},{},{}", transformedPosition.x ,transformedPosition.y
                         // ,transformedPosition.z));
                         double horizonalDistance = std::hypot(transformedPosition.x, transformedPosition.y);
                         double yawAngle = -std::atan2(transformedPosition.y, transformedPosition.x) - glm::half_pi<double>();
                         if(yawAngle < 0)
                             yawAngle += glm::two_pi<double>();
                         const auto expr1 =
                             std::sqrt(square(bulletSpeed) * square(bulletSpeed) -
                                       2 * g * transformedPosition.z * square(bulletSpeed) - square(horizonalDistance * g)) /
                             square(g);

                         const auto expr2 = std::sqrt(square(bulletSpeed / g) - transformedPosition.z / g - expr1);
                         auto expr3 = 2 * glm::root_two<double>() * horizonalDistance *
                             ((-2 * g * transformedPosition.z + 2 * std::pow(bulletSpeed, 2)) * expr2 - square(g) * cube(expr2));

                         expr3 /= (double)(4 * bulletSpeed);
                         expr3 /= (square(transformedPosition.x) + square(transformedPosition.y) + square(transformedPosition.z));

                         auto expr4 = 2 * glm::root_two<double>() *
                             (-square(horizonalDistance) * g * expr2 + square(transformedPosition.z) * g * expr2 -
                              2 * transformedPosition.z * square(bulletSpeed) * expr2 +
                              square(g) * transformedPosition.z * cube(expr2));

                         expr4 /= (double)((-4) * bulletSpeed);
                         expr4 /= (square(transformedPosition.x) + square(transformedPosition.y) + square(transformedPosition.z));
                         double pitchAngle = std::atan2(expr4, expr3);
                         pitchAngle = (pitchAngle < glm::quarter_pi<double>() / 4) ? pitchAngle :
                                                                                     (glm::half_pi<double>() / 2 - pitchAngle);
                         // logInfo(fmt::format("YawAngle: {},PitchAngle: {}", yawAngle, pitchAngle));
                         double currentYawAngle = std::atan2(forwardVector.x, forwardVector.y);
                         if(currentYawAngle < 0.0)
                             currentYawAngle += glm::two_pi<double>();
                         double currentPitchAngle = std::atan2(forwardVector.z, std::hypot(forwardVector.x, forwardVector.y));

                         // logInfo(fmt::format("CurrentYawAngle: {},CurrentPitchAngle: {}",currentYawAngle,
                         // currentPitchAngle)); double prec = mConfig.precision;
                         double prec = 0.001;
                         //                         logInfo(fmt::format("Prec:{}", prec));
                         double diffPitch = pitchAngle - currentPitchAngle, diffYaw = yawAngle - currentYawAngle;
                         if(diffPitch > glm::pi<double>())
                             diffPitch -= glm::two_pi<double>();
                         if(diffPitch < -glm::pi<double>())
                             diffPitch += glm::two_pi<double>();

                         if(diffYaw > glm::pi<double>())
                             diffYaw -= glm::two_pi<double>();
                         if(diffYaw < -glm::pi<double>())
                             diffYaw += glm::two_pi<double>();

                         bool ifShoot = (std::abs(diffPitch) < prec) && (std::abs(diffYaw) < prec);

                         if(yawAngle > glm::pi<double>())
                             yawAngle -= glm::two_pi<double>();

                         // std::cout<<"Gun X "<<positionOfReferenceGun.raw().x<<" Y "<<positionOfReferenceGun.raw().y<<" Z
                         // "<<positionOfReferenceGun.raw().z<<std::endl; std::cout<<"Ground X "<<transformedPosition.x<<" Y
                         // "<<transformedPosition.y<<" Z "<<transformedPosition.z<<std::endl;
                         std::cout << "Yaw:" << glm::degrees(yawAngle) << ", Pitch:" << glm::degrees(pitchAngle) << std::endl;
                         // std::cout<<"current Yaw "<<glm::degrees(currentYawAngle)<<" Pitch
                         // "<<glm::degrees(currentPitchAngle)<<std::endl;
                         std::cout << "Solve "
                                   << (static_cast<double>(Clock::now().time_since_epoch().count()) / Clock::period::den)
                                   << std::endl;

                         yawAngle += -diffYaw + copysign(std::min(0.03 * rand() / RAND_MAX, std::abs(diffYaw)), diffYaw);
                         pitchAngle += -diffPitch + copysign(std::min(0.03 * rand() / RAND_MAX, std::abs(diffPitch)), diffPitch);

                         sendAll(set_target_info_atom_v, yawAngle, pitchAngle, ifShoot);
                     }
                 },
                 [this](update_head_atom, Identifier key) { mHeadKey = key; },
                 [this](update_posture_atom, Identifier key) { mIMUKey = key; } };
    }
};

HUB_REGISTER_CLASS(AngleSolver);
