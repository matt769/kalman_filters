//
// Created by matt on 20/01/2021.
//

#include <iostream>

#include <Eigen/Core>

#include "simple_kf.h"
//#include "state.h"
#include "kalman_filter.h"

Eigen::Matrix<double, 4, 1> Process(const Eigen::Matrix<double, 4, 1>& x) {
    return x;
}


int main() {

    const Eigen::Vector2d z(1.0, 2.0);

    std::cout << "Simple kalman filter\n";
    kf::SimpleKalmanFilter skf;
    for (int i = 0; i < 10; i++) {
        std::cout << "Iteration: " << i << '\n';
        std::cout << "Predict: " << skf.Predict().transpose() << '\n';
        std::cout << "Measurement: " << (z * (double)i).transpose() << "\n";
        std::cout << "Update: " << skf.Update(z * (double)i).transpose() << "\n\n";
    }

    std::cout << "Templated kalman filter 1\n";

    kf1::KalmanFilter<4> kf1;
    kf1::ProcessModel<4> process = [](const Eigen::Matrix<double, 4, 1>& x) -> Eigen::Matrix<double, 4, 1> {
        Eigen::Matrix<double, 4, 4> F = Eigen::Matrix<double, 4, 4>::Identity();
        F(0, 2) = 0.1; // px' = px + vx*dt
        F(1, 3) = 0.1; // py' = py + vy*dt
        return F * x;
    };
    kf1::MeasurementModel<4, 2> measurement_model = [](const Eigen::Matrix<double, 4, 1>& x) -> Eigen::Matrix<double, 2, 1> {
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

    for (int i = 0; i < 10; i++) {
        std::cout << "Iteration: " << i << '\n';
        kf1.Predict(process, Q);
        std::cout << "Predict: " << kf1.GetState().transpose() << '\n';
        kf1.Update<2>(measurement_model, z * (double)i, R);
        std::cout << "Measurement: " << (z * (double)i).transpose() << "\n";
        std::cout << "Update: " << kf1.GetState().transpose() << "\n\n";
    }

    ////////////////////////////////////////////////////////////////////

    std::cout << "Templated kalman filter 2\n";

    kf2::KalmanFilter<4, 2>::ProcessModel process_model_2 = [](const Eigen::Matrix<double, 4, 1>& x) -> Eigen::Matrix<double, 4, 1> {
        Eigen::Matrix<double, 4, 4> F = Eigen::Matrix<double, 4, 4>::Identity();
        F(0, 2) = 0.1; // px' = px + vx*dt
        F(1, 3) = 0.1; // py' = py + vy*dt
        return F * x;
    };
    kf2::KalmanFilter<4, 2>::MeasurementModel measurement_model_2 = [](const Eigen::Matrix<double, 4, 1>& x) -> Eigen::Matrix<double, 2, 1> {
        Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
        // Measure position only
        H(0, 0) = 1.0;
        H(1, 1) = 1.0;
        return H * x;
    };

    kf2::KalmanFilter<4, 2> kf2(process_model_2, measurement_model_2);

    for (int i = 0; i < 10; i++) {
        kf2.Predict(Q);
        kf2.Update(z * (double)i, R);
    }
    std::cout << "Update: " << kf1.GetState().transpose() << "\n\n";


    return 0;
}
