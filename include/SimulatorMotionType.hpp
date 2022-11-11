#pragma once
#include "Transform.hpp"
#include <random>

using MotionState = Transform<FrameOfRef::Robot, FrameOfRef::Ground, true>;

enum class TargetType { Infantry, Hero, BalancedInfantry, Sentry, Outpost, BaseClosed, BaseExpanded, Fans };

enum class TargetMotionType { Static, Spinning, Translate2D, Translate3D, LargeCircle, Fans, Sentry };

enum class SourceMotionType { Static, Vibration, Translate2D, Translate3D, Sentry, UAV };

class MotionController {
public:
    virtual void step(Transform<FrameOfRef::Robot, FrameOfRef::Ground, true>& motionState, double dt) = 0;
    virtual ~MotionController() = default;
};

class StaticMotionController final : public MotionController {
public:
    void step(Transform<FrameOfRef::Robot, FrameOfRef::Ground, true>& motionState, double dt) override;
};

class SpinMotionController final : public MotionController {
    double mSpinningSpeed;

public:
    explicit SpinMotionController(const double spinningSpeed);
    void step(Transform<FrameOfRef::Robot, FrameOfRef::Ground, true>& motionState, double dt) override;
};

class LargeCircleMotionController final : public MotionController {
    double mSpinningSpeed;

public:
    explicit LargeCircleMotionController(const double spinningSpeed);
    void step(Transform<FrameOfRef::Robot, FrameOfRef::Ground, true>& motionState, const double dt) override;
};

class SentryMotionController final : public MotionController {
    bool mMovingDirection = false;
    static constexpr double speed = 0.5f;

public:
    void step(Transform<FrameOfRef::Robot, FrameOfRef::Ground, true>& motionState, const double dt) override;
};

class Translate2DMotionController final : public MotionController {
    static constexpr double alpha = 0.001;

    std::default_random_engine mGenerator;
    double mMaxV;
    std::normal_distribution<double> mDistribution;
    double mSpeedX, mSpeedZ, mT = 0.0;
    double mSmoothX = 0.0, mSmoothZ = 0.0;

    void updateSpeed(const double dt);

public:
    explicit Translate2DMotionController(const double maxV);
    void step(Transform<FrameOfRef::Robot, FrameOfRef::Ground, true>& motionState, const double dt) override;
};