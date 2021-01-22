//
// Created by matt on 20/01/2021.
//

#ifndef SIMPLE_UKF_SIMPLE_KF_H
#define SIMPLE_UKF_SIMPLE_KF_H

#include <Eigen/Core>

// TODO want a nice way of charting or tracking state?
namespace kf {

/** @class A simple but fully specified KF. Discrete and constant time steps.
 *
 */
class SimpleKalmanFilter {
public:
    SimpleKalmanFilter();
    Eigen::Matrix<double, 4, 1> Predict();
    Eigen::Matrix<double, 4, 1> Update(const Eigen::Matrix<double, 2, 1>& z);

private:
    Eigen::Matrix<double, 4, 1> x_; // px, py, vx, vy
    Eigen::Matrix<double, 4, 4> P_;
    Eigen::Matrix<double, 4, 4> F_;
    Eigen::Matrix<double, 4, 4> Q_;
//    Eigen::Matrix<double, ?, ?> B_;
//    Eigen::Matrix<double, ?, ?> u; // control, not used
    Eigen::Matrix<double, 2, 4> H_;
    Eigen::Matrix<double, 2, 2> R_;
//    Eigen::Matrix<double, 2, 1> z; // measurement
    static constexpr double dt = 1.0;

    // Not required as member variables, but might want to inspect so let's keep
    Eigen::Matrix<double, 2, 1> z_hat_;
    Eigen::Matrix<double, 2, 1> y_;
    Eigen::Matrix<double, 2, 2> S_;
    Eigen::Matrix<double, 4, 2> K_;

};

}

#endif //SIMPLE_UKF_SIMPLE_KF_H
