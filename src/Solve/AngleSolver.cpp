#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include "HeadInfo.hpp"
#include "PostureData.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>

struct AngleSolverSettings final {};

template <class Inspector>
bool inspect(Inspector& f, AngleSolverSettings& x) {
    return f.object(x).fields();
}

class AngleSolver final : public HubHelper<caf::event_based_actor, AngleSolverSettings, shoot_atom, set_target_posture_atom> {
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
                     double yawAngle = 0;
                     const double g = 9.80, bulletSpeed = 15.00, pi = std::atan(1) * 4;
                     Vector<UnitType::Distance, FrameOfReference::Gun> positionOfReferenceGun (data.value().center.value().raw());
                     glm::dvec3 transformedPosition = {0,0,0};
                     std::cout << positionOfReferenceGun.raw().x << " " << -positionOfReferenceGun.raw().z << " " << positionOfReferenceGun.raw().y << std::endl;
                     if((dataHeadinfo.has_value())&&(dataPosture.has_value())) {
                         Vector<UnitType::Distance, FrameOfReference::Robot> positionOfReferenceRobot = dataHeadinfo.value().transform(positionOfReferenceGun);
                         Vector<UnitType::Distance, FrameOfReference::Ground> positionOfReferenceGround = dataPosture.value().postureOfRobot(positionOfReferenceRobot);
                         transformedPosition = { positionOfReferenceGround.raw().x, -positionOfReferenceGround.raw().z,positionOfReferenceGround.raw().y };
                     }
                     
                      
                     std::cout << transformedPosition.x << " " << transformedPosition.y << " " << transformedPosition.z<<std::endl;
                     double horizonalDistance = std::sqrt(std::pow(transformedPosition.x, 2) + std::pow(transformedPosition.y, 2));
                     yawAngle = std::atan2(transformedPosition.y, transformedPosition.x) - pi / 2;
                     while(yawAngle < 0)
                         yawAngle += 2 * pi;
                     while(yawAngle >= 2 * pi)
                         yawAngle -= 2 * pi;
                     std::cout << "Yawangle: " << yawAngle <<  std::endl;
                     double expr1 = std::sqrt(std::pow(bulletSpeed, 4) - 2 * g * transformedPosition.z * std::pow(bulletSpeed, 2) -std::pow(horizonalDistance * g, 2)) /std::pow(g, 2);

                     double expr2 = std::sqrt(std::pow(bulletSpeed / g, 2) - transformedPosition.z / g - expr1);
                     double expr3 = 2 * std::sqrt(2) *horizonalDistance*((-2 * g * transformedPosition.z + 2 * std::pow(bulletSpeed, 2)) * expr2 - std::pow(g, 2) * std::pow(expr2, 3));
                     
                     expr3 /= (double)(4 * bulletSpeed);
                     expr3 /= (std::pow(transformedPosition.x, 2)+ std::pow(transformedPosition.y, 2)+std::pow(transformedPosition.z, 2));

                     double expr4 = 2 * std::sqrt(2) * (-std::pow(horizonalDistance, 2) * g * expr2 +
                         std::pow(transformedPosition.z, 2) * g * expr2 - 2 * transformedPosition.z * std::pow(bulletSpeed, 2) * expr2 +
                         std::pow(g, 2) * transformedPosition.z* std::pow(expr2, 3)) ;

                     expr4 /= (double)((-4) * bulletSpeed);
                     expr4 /= (std::pow(transformedPosition.x, 2) + std::pow(transformedPosition.y, 2) +
                               std::pow(transformedPosition.z, 2));
//                     std::cout << "Expr3: " << expr3 << std::endl;
//                     std::cout << "Expr4: " << expr4 << std::endl;
                     double pitchAngle = std::atan2(expr4,expr3);
                     pitchAngle = (pitchAngle < pi / 4) ? pitchAngle : (pi / 2 - pitchAngle);
                     std::cout << "Pitchangle: " << pitchAngle << std::endl;
                     sendAll(set_target_posture_atom_v, yawAngle,pitchAngle);
                     sendAll(shoot_atom_v, true);
                 },
                 [this](update_head_atom, Identifier key) { mHeadKey = key; },
                 [this](update_posture_atom, Identifier key) { mIMUKey = key; } };
    }
};

HUB_REGISTER_CLASS(AngleSolver);
