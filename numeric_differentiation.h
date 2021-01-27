//
// Created by matt on 24/01/2021.
//

#ifndef KALMAN_FILTERS_NUMERIC_DIFFERENTIATION_H_
#define KALMAN_FILTERS_NUMERIC_DIFFERENTIATION_H_

#include <cstddef>
#include <cmath>

#include <Eigen/Core>

namespace numeric_differentiation {

// sqrt isn't constexpr :(
constexpr double getEpsilon() {
  return std::sqrt(std::numeric_limits<double>::epsilon());
};

template <size_t num_params, size_t num_outputs>
Eigen::Matrix<double, num_outputs, num_params>
CalculateJacobian(const Eigen::Matrix<double, num_params, 1> &in,
                  std::function<Eigen::Matrix<double, num_outputs, 1>(
                      const Eigen::Matrix<double, num_params, 1> &)>
                      f) {
  Eigen::Matrix<double, num_outputs, num_params> J;
  const double e = getEpsilon();
  // perturb input and measure change in function output
  for (size_t idx = 0; idx < num_params; ++idx) {
    Eigen::Matrix<double, num_params, 1> ev =
        Eigen::Matrix<double, num_params, 1>::Zero();
    ev(idx) += e;
    Eigen::Matrix<double, num_params, 1> in_minus_e = in - ev;
    Eigen::Matrix<double, num_params, 1> in_plus_e = in + ev;
    Eigen::Matrix<double, num_outputs, 1> out_minus_e = f(in_minus_e);
    Eigen::Matrix<double, num_outputs, 1> out_plus_e = f(in_plus_e);
    J.col(idx) = (out_plus_e - out_minus_e) / (2 * e);
  }
  return J;
}

} // namespace numeric_differentiation

#endif // KALMAN_FILTERS_NUMERIC_DIFFERENTIATION_H_
