#pragma once
#include <opencv2/opencv.hpp>
#include <utility>

struct PIDParameters final {
    double kp, ki, kd;
};

class PIDSimulator final {
    PIDParameters mParameters;
    double mCurrent = 0.0;
    double mLastError = 0.0;
    double mSumError = 0.0;

public:
    explicit PIDSimulator(const PIDParameters& params) : mParameters{ params } {}
    [[nodiscard]] std::pair<double, double> step(double dt, double target, double maxV) noexcept;
};

void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color);

enum class RunStatus { running, normalExit, failureExit };

extern RunStatus globalStatus;

namespace caf {
    class local_actor;
}

void terminateSystem(caf::local_actor& actor, bool success);
