#include "SimulatorMotionType.hpp"
#include <algorithm>

#include "SuppressWarningBegin.hpp"

#include <glm/ext/matrix_transform.hpp>

#include "SuppressWarningEnd.hpp"

void StaticMotionController::step(MotionState& motionState, double dt) {}

SpinMotionController::SpinMotionController(const double spinningSpeed) : mSpinningSpeed(spinningSpeed) {}

void SpinMotionController::step(MotionState& motionState, double dt) {
    motionState =
        glm::rotate(glm::identity<glm::dmat4>(), glm::two_pi<double>() * mSpinningSpeed * dt, glm::dvec3{ 0.0, 1.0, 0.0 }) *
        motionState.val;
}

LargeCircleMotionController::LargeCircleMotionController(const double spinningSpeed) : mSpinningSpeed(spinningSpeed) {}

void LargeCircleMotionController::step(MotionState& motionState, double dt) {
    motionState =
        glm::rotate(glm::identity<glm::dmat4>(), glm::two_pi<double>() * mSpinningSpeed * dt, glm::dvec3{ 0.0, 1.0, 0.0 }) *
        motionState.val;
}

void SentryMotionController::step(MotionState& motionState, double dt) {
    auto& position = motionState.displacement();
    if(position.val.x > 2.0)
        mMovingDirection = false;
    else if(position.val.x < -2.0)
        mMovingDirection = true;

    position.val.x += (mMovingDirection ? speed * dt : -speed * dt);
}

void Translate2DMotionController::updateSpeed(const double dt) {
    mT += dt;
    if(mT >= 2.0) {                                                      // Random generation velecity every two seconds
        mSpeedX = std::clamp(mDistribution(mGenerator), -mMaxV, mMaxV);  // Limit the speed of random generation
        mSpeedZ = std::clamp(mDistribution(mGenerator), -mMaxV, mMaxV);

        mT -= 2.0;
    }

    mSmoothX = (1.0 - alpha) * mSmoothX + alpha * mSpeedX;
    mSmoothZ = (1.0 - alpha) * mSmoothZ + alpha * mSpeedZ;
}

Translate2DMotionController::Translate2DMotionController(const double maxV) : mMaxV{ maxV }, mDistribution{ 0, maxV / 3.0 } {
    updateSpeed(2.0 + 1e-5);
}

void Translate2DMotionController::step(MotionState& motionState, const double dt) {
    updateSpeed(dt);

    motionState = glm::translate(glm::identity<glm::dmat4>(), glm::dvec3{ mSmoothX * dt, 0.0, mSmoothZ * dt }) * motionState.val;
}