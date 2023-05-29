// Copyright 2022 Chen Jun

#pragma once
#include <Eigen/Dense>
#include <functional>

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter() = default;

    using NonlinearFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
    using JacobianFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>;

    explicit ExtendedKalmanFilter(const NonlinearFunc& f, const NonlinearFunc& h, const JacobianFunc& Jf, const JacobianFunc& Jh,
                                  const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& P0);

    // Set the initial state
    void setState(const Eigen::VectorXd& x0);

    // Compute a predicted state
    Eigen::MatrixXd predict();

    // Update the estimated state based on measurement
    Eigen::MatrixXd update(const Eigen::VectorXd& z);

    // Process nonlinear vector function
    NonlinearFunc f;
    // Observation nonlinear vector function
    NonlinearFunc h;
    // Jacobian of f()
    JacobianFunc Jf;
    Eigen::MatrixXd F;
    // Jacobian of h()
    JacobianFunc Jh;
    Eigen::MatrixXd H;
    // Process noise covariance matrix
    Eigen::MatrixXd Q;
    // Measurement noise covariance matrix
    Eigen::MatrixXd R;

    // Priori error estimate covariance matrix
    Eigen::MatrixXd PPri;
    // Posteriori error estimate covariance matrix
    Eigen::MatrixXd PPost;

    // Kalman gain
    Eigen::MatrixXd K;

    // System dimensions
    int n;

    // N-size identity
    Eigen::MatrixXd I;

    // Priori state
    Eigen::VectorXd xPri;
    // Posteriori state
    Eigen::VectorXd xPost;
};
