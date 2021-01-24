//
// Created by matt on 20/01/2021.
//

#ifndef KALMAN_FILTERS_STATE_H
#define KALMAN_FILTERS_STATE_H

#include <Eigen/Core>

namespace kf {

template<size_t STATE_SIZE>
struct State {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using StateVector = Eigen::Matrix<double, STATE_SIZE, 1>;
  using CovMatrix = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;

  State(const StateVector& state, const CovMatrix& cov)
      : x(state), P(cov) {}

  State(const StateVector& state)
      : State(x, CovMatrix::Identity()) {}

  State()
      : State(StateVector::Zero(), CovMatrix::Identity()) {}

  StateVector x;
  CovMatrix P;
};

} // namespace kf

#endif //KALMAN_FILTERS_STATE_H
