//
// Created by matt on 20/01/2021.
//

#ifndef KALMAN_FILTERS_SIMPLE_KF_H
#define KALMAN_FILTERS_SIMPLE_KF_H

#include <Eigen/Core>

namespace kf0 {

/** @class A simple but fully specified KF. Discrete and constant time steps.
 *
 */
class KalmanFilter {
public:
  KalmanFilter();
  Eigen::Matrix<double, 4, 1> Predict();
  Eigen::Matrix<double, 4, 1> Update(const Eigen::Matrix<double, 2, 1> &z);
  Eigen::Matrix<double, 4, 1> GetState() const { return x_; };
  Eigen::Matrix<double, 4, 4> GetCov() const { return P_; };
  void SetState(const Eigen::Matrix<double, 4, 1>& new_state) { x_ = new_state; };
  void SetCov(const Eigen::Matrix<double, 4, 4>& new_cov) { P_ = new_cov; };

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
  static constexpr double dt = 0.1;

  // Not required as member variables, but might want to inspect so let's keep
  Eigen::Matrix<double, 2, 1> z_hat_;
  Eigen::Matrix<double, 2, 1> y_;
  Eigen::Matrix<double, 2, 2> S_;
  Eigen::Matrix<double, 4, 2> K_;
};

} // namespace kf0

#endif // KALMAN_FILTERS_SIMPLE_KF_H
