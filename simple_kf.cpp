//
// Created by matt on 21/01/2021.
//

#include "simple_kf.h"

#include <iostream>

#include <Eigen/Core>
#include <Eigen/LU>


// TODO want a nice way of charting or tracking state?
namespace kf {

SimpleKalmanFilter::SimpleKalmanFilter() {
  // State, position (x, y) and velocity (x, y)
  x_.setZero();
  // Initialise covariance with high values
  P_.setZero();
  P_.diagonal() = Eigen::Vector4d::Constant(100.0);
  // Process model
  // v' = v
  // p' = p + v*dt
  F_.setIdentity();
  F_(0, 2) = dt; // px' = px + vx*dt
  F_(1, 3) = dt; // py' = py + vy*dt
  // Process noise
  Q_ = Eigen::Matrix<double, 4, 4>::Identity() * 2.0; // TODO placeholder
  // Measurement model
  // Measure position
  H_.setZero();
  H_(0, 0) = 1.0;
  H_(1, 1) = 1.0;
  // Measurement noise
  R_.setIdentity(); // TODO placeholder

//    std::cout << "F: " << '\n';
//    std::cout << F_ << '\n';
//    std::cout << "x: " << '\n';
//    std::cout << x_.transpose() << '\n';
//    std::cout << "P: " << '\n';
//    std::cout << P_ << '\n';
//    std::cout << "Q: " << '\n';
//    std::cout << Q_ << '\n';
//    std::cout << "H: " << '\n';
//    std::cout << H_ << '\n';
//    std::cout << "R: " << '\n';
//    std::cout << R_ << "\n\n";


}

Eigen::Matrix<double, 4, 1> SimpleKalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;

//    std::cout << "Predict\n";
//    std::cout << "F: " << '\n';
//    std::cout << F_ << '\n';
//    std::cout << "x: " << '\n';
//    std::cout << x_.transpose() << '\n';
//    std::cout << "P: " << '\n';
//    std::cout << P_ << "\n\n";
  return x_;
}

Eigen::Matrix<double, 4, 1> SimpleKalmanFilter::Update(const Eigen::Matrix<double, 2, 1>& z) {
  z_hat_ = H_ * x_;
  y_ = z - z_hat_;
  S_ = H_ * P_ * H_.transpose() + R_;
  K_ = P_ * H_.transpose() * S_.inverse();
  x_ = x_ + K_ * y_;
  P_ = (Eigen::Matrix4d::Identity() - K_ * H_) * P_;

//    std::cout << "Update\n";
//    std::cout << "z: " << '\n';
//    std::cout << z.transpose() << '\n';
//    std::cout << "z_hat: " << '\n';
//    std::cout << z_hat_.transpose() << '\n';
//    std::cout << "y: " << '\n';
//    std::cout << y_.transpose() << '\n';
//    std::cout << "S: " << '\n';
//    std::cout << S_ << '\n';
//    std::cout << "K: " << '\n';
//    std::cout << K_ << '\n';
//    std::cout << "x: " << '\n';
//    std::cout << x_.transpose() << '\n';
//    std::cout << "P: " << '\n';
//    std::cout << P_ << "\n\n";

  return x_;
}

}