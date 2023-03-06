#pragma once
#include "SuppressWarningBegin.hpp"

#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>

#include "SuppressWarningEnd.hpp"

#include <utility>
#include <vector>

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
    void reset(const double current) {
        mCurrent = current;
    }
    // period = -1.0: disabled
    // period > 0.0: domain is [0,period)
    // return (position,velocity)
    [[nodiscard]] std::pair<double, double> step(double dt, double target, double maxV, double period = -1.0) noexcept;
};

void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness = 1);

enum class RunStatus { running, normalExit, failureExit };

extern RunStatus globalStatus;

namespace caf {
    class local_actor;
}

void terminateSystem(caf::local_actor& actor, bool success);

extern std::string globalConfigName;
void appendTestResult(const std::string& message);
std::vector<uint32_t> solveKM(uint32_t n, uint32_t m, const std::vector<double>& w);

// width < height
// angle = 0
// 1 width 2
// height  height
// 0 width 3
// angle = 90
// 0 height 1
// width    width
// 3 height 2

void boxRect(std::vector<cv::Point2f>& res, const cv::RotatedRect& rect);

// a,b,c mustn't be on the same line or on the same point
glm::dvec3 circleCenter(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c);

// pair[center,radius]
std::pair<glm::dvec3, double> CircleFitByTaubin(const std::vector<glm::dvec3>& pts);

// pair[k,m]
std::pair<double, double> FitLine(const std::vector<std::pair<double, double>>& pts);

template <typename T>
T square(const T& n) {
    return n * n;
}

template <typename Seq>
auto avg(const Seq& array) {
    typename Seq::value_type res = 0;
    for(const auto& item : array)
        res += item;
    return res / array.size();
}

template <typename Seq, typename T>
auto Std(const Seq& array, T avg) {
    typename Seq::value_type res = 0;
    for(const auto& item : array)
        res += square(item - avg);
    return std::sqrt(res / array.size());
}

std::complex<double> sqrtN(const std::complex<double>& x, double n);

double ferrari(std::complex<double> a, std::complex<double> b, std::complex<double> c, std::complex<double> d,
               std::complex<double> e);

// tuple[time,yawAngle,pitchAngle]
std::tuple<double, double, double> solveWithoutAirDrag(glm::dvec3 targetPos, glm::dvec3 targetVel);