#include "Timer.hpp"
#include <eigen3/Eigen/Dense>
#include <glm/glm.hpp>

class KalmanFilter {
public:
    glm::dvec3 getPredictedVel();

private:
    bool mInitFlag = false;
    int32_t mCntWrongData = 0;
    glm::dvec3 mLastPos;
    TimePoint mLastTimePoint;
    glm::dvec3 mPredictedVel;
    Eigen::VectorXd mX;  // State vector(Position & Velocity)
    Eigen::MatrixXd mF;  // State transform mat
    Eigen::MatrixXd mP;  // State covariance mat
    Eigen::MatrixXd mQ;  // Process covariance mat
    Eigen::MatrixXd mH;  // Measurement mat
    Eigen::MatrixXd mR;  // Measurement covariance mat

    void setX(const glm::dvec3& measuredPos);

    void initialKalmanFilter(const glm::dvec3& measuredPos, const TimePoint& curTimePoint);
    void Prediction();
    void UpdateMeasurement(const Eigen::VectorXd z);
    void KmFilter(const glm::dvec3& pos, double dt);
    void RunFilter(const glm::dvec3& measuredPos, const TimePoint& curTimePoint);

    glm::dvec3 filterWrongData(const glm::dvec3& measuredPos);
};