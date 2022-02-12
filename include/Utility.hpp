#pragma once
#include <opencv2/opencv.hpp>
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
    void reset(double current) {
        mCurrent = current;
    }
    // period = -1.0: disabled
    // period > 0.0: domain is [0,period)
    // return (position,velocity)
    [[nodiscard]] std::pair<double, double> step(double dt, double target, double maxV, double period = -1.0) noexcept;
};

void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color);

enum class RunStatus { running, normalExit, failureExit };

extern RunStatus globalStatus;

namespace caf {
    class local_actor;
}

void terminateSystem(caf::local_actor& actor, bool success);

class Crc {
    const static uint8_t CRC8_TAB[256];
    const static uint16_t CRC16_TAB[256];

public:
    const static uint8_t CRC8_INIT;
    const static uint16_t CRC16_INIT;

    static uint8_t Get_CRC8_Check_Sum(uint8_t* pchMessage, uint32_t dwLength, uint8_t ucCRC8);
    static bool VerifyCrc8CheckSum(uint8_t* pchMessage, uint32_t dwLength);

    static uint16_t Get_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);
    static bool VerifyCrc16CheckSum(uint8_t* pchMessage, uint32_t dwLength);
};

void appendTestResult(const std::string& message);
std::vector<uint32_t> solveKM(uint32_t n, uint32_t m, const std::vector<double>& w);
