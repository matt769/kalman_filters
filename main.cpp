//
// Created by matt on 20/01/2021.
//

#include <iostream>

#include <Eigen/Core>

#include "simple_kf.h"

int main() {

    kf::SimpleKalmanFilter skf;
//    Eigen::Vector4d x = skf.Predict();
//    std::cout << x.transpose() << '\n';

    Eigen::Vector2d z(1.0, 2.0);
//    Eigen::Vector4d x2 = skf.Update(z);
//    std::cout << x2.transpose() << '\n';

    for (int i = 0; i < 10; i++) {
        std::cout << skf.Predict().transpose() << '\n';
        std::cout << skf.Update(z * (double)i).transpose() << '\n';
    }


    return 0;
}
