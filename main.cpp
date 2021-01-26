//
// Created by matt on 20/01/2021.
//

#include <iostream>

#include <Eigen/Core>

#include "simple_kf.h"
#include "standard_kf.h"
#include "extended_kf.h"
#include "unscented_kf.h"
#include "system.h"
#include "numeric_differentiation.h"


int main() {

  const int iterations = 3;
  using SS = systems::SimpleSystem;
  const SS::MeasurementVector z(1.0, 2.0);

  ////////////////////////////////////////////////////////////////////
  std::cout << "Simple kalman filter\n";
  kf0::KalmanFilter skf;
  skf.SetCov(systems::SimpleSystem::StateMatrix::Identity());
  std::cout << "Intial state:\n";
  std::cout << skf.GetState().transpose() << '\n';
  std::cout << skf.GetCov() << '\n';
  for (int i = 0; i < iterations; i++) {
    std::cout << "Iteration: " << i << '\n';
    std::cout << "Predict: " << skf.Predict().transpose() << '\n';
    std::cout << skf.GetCov() << '\n';
    std::cout << "Measurement: " << (z * (double) i).transpose() << "\n";
    std::cout << "Update: " << skf.Update(z * (double) i).transpose() << "\n";
    std::cout << skf.GetCov() << "\n\n";
  }

  ////////////////////////////////////////////////////////////////////
  std::cout << "Templated kalman filter 1\n";

  kf1::KalmanFilter<4> kf1;
  kf1::ProcessModel<4> process = [](const Eigen::Matrix<double, 4, 1>& x) -> Eigen::Matrix<double, 4, 1> {
    Eigen::Matrix<double, 4, 4> F = Eigen::Matrix<double, 4, 4>::Identity();
    F(0, 2) = 0.1; // px' = px + vx*dt
    F(1, 3) = 0.1; // py' = py + vy*dt
    return F * x;
  };
  kf1::MeasurementModel<4, 2> measurement_model = [](
      const Eigen::Matrix<double, 4, 1>& x) -> Eigen::Matrix<double, 2, 1> {
    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    // Measure position only
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    return H * x;
  };

//    kf1.SetState(Eigen::Vector4d(0.5,0.0,0.0,0.2));
  kf1.SetCov(Eigen::Matrix4d::Identity() * 100.0);
  Eigen::Matrix4d Q = Eigen::Matrix4d::Identity() * 2.0;
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity();

//  std::cout << kf1.GetState().transpose() << '\n';
//  std::cout << kf1.GetCov() << '\n';
//  for (int i = 0; i < iterations; i++) {
//    std::cout << "Iteration: " << i << '\n';
//    kf1.Predict(process, Q);
//    std::cout << "Predict: " << kf1.GetState().transpose() << '\n';
//    std::cout << kf1.GetCov() << '\n';
//    kf1.Update<2>(measurement_model, z * (double) i, R);
//    std::cout << "Measurement: " << (z * (double) i).transpose() << "\n";
//    std::cout << "Update: " << kf1.GetState().transpose() << "\n\n";
//    std::cout << kf1.GetCov() << '\n';
//  }

  ////////////////////////////////////////////////////////////////////

  std::cout << "Templated kalman filter 2\n";

  kf2::KalmanFilter<4, 2>::ProcessModel process_model_2 = [](
      const Eigen::Matrix<double, 4, 1>& x) -> Eigen::Matrix<double, 4, 1> {
    Eigen::Matrix<double, 4, 4> F = Eigen::Matrix<double, 4, 4>::Identity();
    F(0, 2) = 0.1; // px' = px + vx*dt
    F(1, 3) = 0.1; // py' = py + vy*dt
    return F * x;
  };
  kf2::KalmanFilter<4, 2>::MeasurementModel measurement_model_2 = [](
      const Eigen::Matrix<double, 4, 1>& x) -> Eigen::Matrix<double, 2, 1> {
    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    // Measure position only
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    return H * x;
  };

  kf2::KalmanFilter<4, 2> kf2(process_model_2, measurement_model_2);
  kf2.SetCov(Eigen::Matrix4d::Identity() * 100.0);

//  for (int i = 0; i < iterations; i++) {
//    std::cout << "Iteration: " << i << '\n';
//    kf2.Predict(Q);
//    std::cout << "Predict: " << kf2.GetState().transpose() << '\n';
//    kf2.Update(z * (double) i, R);
//    std::cout << "Measurement: " << (z * (double) i).transpose() << "\n";
//    std::cout << "Update: " << kf2.GetState().transpose() << "\n\n";
//  }

  ////////////////////////////////////////////////////////////////////
  std::cout << "Templated kalman filter 3\n";

  kf3::KalmanFilter<4, 2, SS> kf_3;
  kf_3.SetCov(Eigen::Matrix4d::Identity() * 100.0);

  for (int i = 0; i < iterations; i++) {
    std::cout << "Iteration: " << i << '\n';
    kf_3.Predict(Q);
    std::cout << "Predict: " << kf_3.GetState().transpose() << '\n';
    kf_3.Update(z * (double) i, R);
    std::cout << "Measurement: " << (z * (double) i).transpose() << "\n";
    std::cout << "Update: " << kf_3.GetState().transpose() << "\n\n";
  }

  ////////////////////////////////////////////////////////////////////
  std::cout << "Templated kalman filter 4\n";

  kf4::KalmanFilter<SS> kf_4;
  kf_4.SetCov(Eigen::Matrix4d::Identity() * 100.0);

  for (int i = 0; i < iterations; i++) {
    std::cout << "Iteration: " << i << '\n';
    kf_4.Predict(Q);
    std::cout << "Predict: " << kf_4.GetState().transpose() << '\n';
    kf_4.Update(z * (double) i, R);
    std::cout << "Measurement: " << (z * (double) i).transpose() << "\n";
    std::cout << "Update: " << kf_4.GetState().transpose() << "\n\n";
  }

  ////////////////////////////////////////////////////////////////////
  std::cout << "Another system\n";

  const Eigen::Vector3d z3(1.0, 2.0, 0.5);
  Eigen::Matrix<double, 6, 6> Q3 = Eigen::Matrix<double, 6, 6>::Identity() * 2.0;
  Eigen::Matrix3d R3 = Eigen::Matrix3d::Identity();

  kf4::KalmanFilter<systems::AnotherSystem> kf_5;
  for (int i = 0; i < iterations; i++) {
    std::cout << "Iteration: " << i << '\n';
    kf_5.Predict(Q3);
    std::cout << "Predict: " << kf_5.GetState().transpose() << '\n';
    kf_5.Update(z3 * (double) i, R3);
    std::cout << "Measurement: " << (z3 * (double) i).transpose() << "\n";
    std::cout << "Update: " << kf_5.GetState().transpose() << "\n\n";
  }

  ////////////////////////////////////////////////////////////////////

  Eigen::Vector4d current_state = Eigen::Vector4d::Zero();

  SS::StateMatrix JF = numeric_differentiation::CalculateJacobian<SS::kStateSize, SS::kStateSize>(current_state, SS::processModel);
  std::cout << JF << "\n\n";
  Eigen::Matrix<double, SS::kMeasurementSize, SS::kStateSize> JH =
      numeric_differentiation::CalculateJacobian<SS::kStateSize, SS::kMeasurementSize>(current_state, SS::measurementModel);
  std::cout << JH << "\n\n";


  ////////////////////////////////////////////////////////////////////
  std::cout << "EKF 1 (with linear system) \n";

  experimental::ekf1::ExtendedKalmanFilter<SS> ekf_1;
  ekf_1.SetCov(SS::StateMatrix::Identity() * 100.0);

  for (int i = 0; i < iterations; i++) {
    std::cout << "Iteration: " << i << '\n';
    ekf_1.Predict(Q);
    std::cout << "Predict: " << ekf_1.GetState().transpose() << '\n';
    ekf_1.Update(z * (double) i, R);
    std::cout << "Measurement: " << (z * (double) i).transpose() << "\n";
    std::cout << "Update: " << ekf_1.GetState().transpose() << "\n\n";
  }

  ////////////////////////////////////////////////////////////////////
  std::cout << "EKF 1 (with non-linear system) \n";

  using NLS = systems::NonLinearSystem;
  experimental::ekf1::ExtendedKalmanFilter<NLS> ekf_2;
  NLS::ProcessNoiseMatrix Q_nl = NLS::ProcessNoiseMatrix::Identity() * 10.0;
  NLS::MeasurementNoiseMatrix R_nl = NLS::MeasurementNoiseMatrix::Identity() * 1.0;

  ekf_2.SetCov(Q_nl);
  for (int i = 0; i < iterations; i++) {
    std::cout << "Iteration: " << i << '\n';
    ekf_2.Predict(Q_nl);
    std::cout << "Predict: " << ekf_2.GetState().transpose() << '\n';
    ekf_2.Update(z, R_nl);
    std::cout << "Measurement: " << z.transpose() << "\n";
    std::cout << "Update: " << ekf_2.GetState().transpose() << "\n\n";
  }

 // velocity is estimated correctly after a few iterations
 // why doesn't heading work out?

  ////////////////////////////////////////////////////////////////////
  std::cout << "Unscented kalman filter 1\n";

  experimental::ukf1::UnscentedKalmanFilter<SS> ukf_1;
  ukf_1.SetCov(SS::StateMatrix::Identity());

  std::cout << ukf_1.GetState().transpose() << '\n';
  std::cout << ukf_1.GetCov() << '\n';
  for (int i = 0; i < iterations; i++) {
    std::cout << "Iteration: " << i << '\n';
    ukf_1.Predict(Q);
    std::cout << "Predict: " << ukf_1.GetState().transpose() << '\n';
    std::cout << ukf_1.GetCov() << '\n';
    ukf_1.Update(z * (double) i, R);
    std::cout << "Measurement: " << (z * (double) i).transpose() << "\n";
    std::cout << "Update: " << ukf_1.GetState().transpose() << "\n";
    std::cout << ukf_1.GetCov() << "\n\n";
  }




  return 0;
}
