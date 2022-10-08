#include "ArmorPredictor.hpp"

constexpr double maxDeltaTime = 0.2;

glm::dvec3 KalmanFilter::getPredictedVel(){
    return predictedVel;
}

void KalmanFilter::initialKalmanFilter() {
    mX.resize(6);
    mF.setIdentity(6, 6);
    mP.setIdentity(6, 6);
    mQ.setIdentity(6, 6);
    mH.resize(3, 6);
    mH << 1.0, 0.0, 0.0,  //
        0.0, 1.0, 0.0,    //
        0.0, 0.0, 1.0;    //
    mR.resize(3, 3);
    mR << 0.01, 0.0, 0.0,  //
        0.0, 0.01, 0.0,    //
        0.0, 0.0, 0.01;
}

void KalmanFilter::Prediction() {
    mX = mF * mX;
    mP = mF * mP * mF.transpose() + mQ;
}

void KalmanFilter::UpdateMeasurement(const Eigen::VectorXd z) {
    const auto y = z - mH * mX;  // Measure
    const auto S = mH * mP * mH.transpose() + mR;
    const auto K = mP * mH.transpose() * S.inverse();  // Kalman Gain
    mX = mX + (K * y);                                 // Optimal estimate
    const auto I = Eigen::MatrixXd::Identity(mX.size(), mX.size());
    mP = (I - K * mH) * mP;
}

void KalmanFilter::KmFilter(const glm::dvec3& pos, double dt) {

    Eigen::MatrixXd inputF(6, 6);
    inputF << 1.0, 0.0, 0.0, dt, 0.0, 0.0,  //
        0.0, 1.0, 0.0, 0.0, dt, 0.0,        //
        0.0, 0.0, 1.0, 0.0, 0.0, dt,        //
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,       //
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,       //
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;       //
    mF << inputF;

    Prediction();
    Eigen::VectorXd measuredZ(3, 1);
    measuredZ << pos.x, pos.y, pos.z;
    UpdateMeasurement(measuredZ);

    mPredictedVec = { mX(3), mX(4), mX(5) };
}

void KalmanFilter::RunFilter(const glm::dvec3& measuredPos, const TimePoint& curTimePoint) {
    double deltaTime = static_cast<double>((curTimePoint - mLastTimePoint).time_since_epoch().count()) / 1e9;
    if(deltaTime > maxDeltaTime) {
        mLastPos = measuredPos;
        mLastTimePoint = curTimePoint;
        return;
    }
}
