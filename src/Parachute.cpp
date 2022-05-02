//
// Created by 16182 on 1/31/2022.
//

#include "Parachute.h"
void DrogueParachute::update(Eigen::Vector3d position, Eigen::Vector3d velocity) {
    (void)position;
    constexpr double min_descent_rate = -3;
    if(velocity.z() < min_descent_rate){
        is_open_ = true;
    }
}

void MainParachute::update(Eigen::Vector3d position, Eigen::Vector3d velocity) {
    if(velocity.z() < 0 && position.z() < deploy_altitude_){
        is_open_ = true;
    }
}

