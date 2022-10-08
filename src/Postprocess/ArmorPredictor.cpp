#include "ArmorPredictor.hpp"

constexpr double maxDeltaTime = 0.2;
constexpr int32_t maxCntWrongData = 5;

constexpr double maxJumpXDist = 0.5;
constexpr double maxJumpYDist = 0.5;
constexpr double maxJumpZDist = 0.5;

constexpr double maxXVel = 3.0;
constexpr double maxYVel = 3.0;
constexpr double maxZVel = 1.0;

glm::dvec3 KalmanFilter::getPredictedVel(){
    return mPredictedVel;
}

void KalmanFilter::initialKalmanFilter(const glm::dvec3& measuredPos, const TimePoint& curTimePoint) {
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

    mLastPos = measuredPos;
    mLastTimePoint = curTimePoint;
    setX(measuredPos);
    mInitFlag = true;
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

    mPredictedVel = { mX(3), mX(4), mX(5) };
    if (mPredictedVel.x > maxXVel){
        mPredictedVel.x = maxXVel;
    }
    if (mPredictedVel.y > maxYVel){
        mPredictedVel.y = maxYVel;
    }
    if (mPredictedVel.z > maxZVel){
        mPredictedVel.z = maxZVel;
    }
}

void KalmanFilter::RunFilter(const glm::dvec3& measuredPos, const TimePoint& curTimePoint) {
    if (!mInitFlag){
        initialKalmanFilter(measuredPos, curTimePoint);
        return;
    }

    double deltaTime = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(curTimePoint - mLastTimePoint).count()) / 1e6;
    if(deltaTime > maxDeltaTime) {      
        initialKalmanFilter(measuredPos, curTimePoint);
        return;
    }
    const auto filterPos = filterWrongData(measuredPos);
    if (!mInitFlag){
        initialKalmanFilter(measuredPos, curTimePoint);
        return;
    }
    KmFilter(filterPos, deltaTime);
}

void KalmanFilter::setX(const glm::dvec3& measuredPos){
    Eigen::VectorXd initX(6, 1);
    initX << measuredPos.x, measuredPos.y, measuredPos.z, 0.0, 0.0, 0.0;
}

glm::dvec3 KalmanFilter::filterWrongData(const glm::dvec3& measuredPos){
    glm::dvec3 filterRes = measuredPos;
    bool isWrongData = false;
    if (mCntWrongData < maxCntWrongData){
        if (std::fabs(measuredPos.x - mLastPos.x) > maxJumpXDist){
            isWrongData = true;
            filterRes.x = mLastPos.x;
        }
        if (std::fabs(measuredPos.y - mLastPos.y) > maxJumpYDist){
            isWrongData = true;
            filterRes.y = mLastPos.y;
        }
        if (std::fabs(measuredPos.z - mLastPos.z) > maxJumpXDist){
            isWrongData = true;
            filterRes.z = mLastPos.z;
        }
    }else{
        mInitFlag = false;
        mCntWrongData = 0;
    }

    if (isWrongData){
        mCntWrongData = 0;
    }else{
        ++mCntWrongData;
    }
    return filterRes;

}