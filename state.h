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

namespace kf_dynamic {

struct State {
  using StateVector = Eigen::VectorXd;
  using CovMatrix = Eigen::MatrixXd;

  State(const StateVector& starting_state, const CovMatrix& starting_cov)
      : x(starting_state), P(starting_cov) {}

  State(const StateVector& starting_state)
      : State(starting_state, CovMatrix::Identity(starting_state.rows(), starting_state.cols())) {}

  StateVector x;
  CovMatrix P;
};

} // namespace kf

#endif //KALMAN_FILTERS_STATE_H
