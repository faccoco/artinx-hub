#include "Timer.hpp"
#include <eigen3/Eigen/Dense>
#include <glm/glm.hpp>

class KalmanFilter {
public:
    void initialKalmanFilter();
    void Prediction();
    void UpdateMeasurement(const Eigen::VectorXd z);
    void KmFilter(const glm::dvec3& pos, double dt);
    void RunFilter(const glm::dvec3& measuredPos, const TimePoint& curTimePoint);

private:
    bool mInitFlag = false;
    int mStep;
    glm::dvec3 mLastPos;
    TimePoint mLastTimePoint;
    Eigen::VectorXd mX;  // State vector(Position Velocity)
    Eigen::MatrixXd mF;  // State transform mat
    Eigen::MatrixXd mP;  // State covariance mat
    Eigen::MatrixXd mQ;  // Process covariance mat
    Eigen::MatrixXd mH;  // Measurement mat
    Eigen::MatrixXd mR;  // Measurement covariance mat
};