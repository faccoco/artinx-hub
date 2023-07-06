// Copyright 2022 Chen Jun

#include "EKF.hpp"
#include <iostream>

ExtendedKalmanFilter::ExtendedKalmanFilter(const NonlinearFunc& f, const NonlinearFunc& h, const JacobianFunc& Jf,
                                           const JacobianFunc& Jh, const VoidMatFunc& UQ, const VecMatFunc& UR,
                                           const Eigen::MatrixXd& P0)
    : f(f), h(h), Jf(Jf), Jh(Jh), updateQ(UQ), updateR(UR), PPost(P0), n(P0.rows()), I(Eigen::MatrixXd::Identity(n, n)), xPri(n),
      xPost(n) {}

void ExtendedKalmanFilter::setState(const Eigen::VectorXd& x0) {
    xPost = x0;
}

Eigen::MatrixXd ExtendedKalmanFilter::predict() {
    F = Jf(xPost), Q = updateQ();
    std::cout << "Q:  " << Q(0, 0) << "\t" << Q(1, 1) << "\t" << Q(2, 2) << "\t" << std::endl;
    xPri = f(xPost);
    PPri = F * PPost * F.transpose() + Q;

    // handle the case when there will be no measurement before the next predict
    xPost = xPri;
    PPost = PPri;

    return xPri;
}

Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd& z) {
    H = Jh(xPri), R = updateR(z);
    std::cout << "R:  " << R(0, 0) << "\t" << R(1, 1) << "\t" << R(2, 2) << "\t" << std::endl;


    K = PPri * H.transpose() * (H * PPri * H.transpose() + R).inverse();
    xPost = xPri + K * (z - h(xPri));
    std::cout << K << std::endl;
    std::cout << H << std::endl;
    PPost = (I - K * H) * PPri;


    return xPost;
}
