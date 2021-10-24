#include "Utility.hpp"
#include <cmath>

std::pair<double, double> PIDSimulator::step(double dt, double target, double maxV) noexcept {
    const auto error = target - mCurrent;
    mSumError += error;
    const auto diff = error - mLastError;
    mLastError = error;
    const auto v = std::fmin(maxV, mParameters.kp * error + mParameters.ki * mSumError + mParameters.kd * diff);
    mCurrent += v * dt;
    return std::make_pair(mCurrent, v);
}
