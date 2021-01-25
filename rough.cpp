//
// Created by matt on 25/01/2021.
//

#include <iostream>

#include <Eigen/Core>

#include "standard_kf.h"
#include "unscented_kf.h"
#include "system.h"

Eigen::Matrix<double, 4, 1> Process(const Eigen::Matrix<double, 4, 1>& x) {
  return x;
}

int main() {

  const int iterations = 2;
  Eigen::Matrix<double, 1, 1> z;
  z(0) = 2.0;

  Eigen::Matrix2d Q = Eigen::Matrix2d::Identity() * 2.0;
  Eigen::Matrix<double, 1, 1> R = Eigen::Matrix<double, 1, 1>::Identity();

  ////////////////////////////////////////////////////////////////////
  std::cout << "Templated kalman filter 4\n";

  kf4::KalmanFilter<systems::TestSystem> kf;
  kf.SetCov(Eigen::Matrix2d::Identity());
  std::cout << "Intial state:\n";
  std::cout << kf.GetState().transpose() << "\n\n";
  std::cout << kf.GetCov() << "\n\n";
  for (int i = 0; i < iterations; i++) {
    std::cout << "Iteration: " << i << "\n\n";
    kf.Predict(Q);
    std::cout << "Predict: " << kf.GetState().transpose() << "\n\n";
    std::cout << kf.GetCov() << "\n\n";
    std::cout << "Measurement: " << (z * (double) i).transpose() << "\n\n";
    kf.Update(z * (double) i, R);
    std::cout << "Update: " << kf.GetState().transpose() << "\n\n";
    std::cout << kf.GetCov() << "\n\n";
  }

  ////////////////////////////////////////////////////////////////////
  std::cout << "Unscented kalman filter 1\n";

  experimental::ukf1::UnscentedKalmanFilter<systems::TestSystem> ukf_1;
  ukf_1.SetCov(Eigen::Matrix2d::Identity());

  std::cout << ukf_1.GetState().transpose() << "\n\n";
  std::cout << ukf_1.GetCov() << "\n\n";
  for (int i = 0; i < iterations; i++) {
    std::cout << "Iteration: " << i << "\n\n";
    ukf_1.Predict(Q);
    std::cout << "Predict: " << ukf_1.GetState().transpose() << "\n\n";
    std::cout << ukf_1.GetCov() << "\n\n";
    std::cout << "Measurement: " << (z * (double) i).transpose() << "\n\n";
    ukf_1.Update(z * (double) i, R);
    std::cout << "Update: " << ukf_1.GetState().transpose() << "\n\n";
    std::cout << ukf_1.GetCov() << "\n\n";
  }


  return 0;
}
