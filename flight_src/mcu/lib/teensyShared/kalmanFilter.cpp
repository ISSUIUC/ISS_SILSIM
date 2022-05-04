
#include "kalmanFilter.h"

#define EIGEN_MATRIX_PLUGIN "MatrixAddons.h"

#include <iostream>
#include <string>
#include <fstream>

#include <string>
#include "GlobalVars.h"

KalmanFilter::KalmanFilter(struct pointers* pointer_struct) {
    gz_L = &pointer_struct->sensorDataPointer->lowG_data.gz;
    gz_H = &pointer_struct->sensorDataPointer->highG_data.hg_az;
    b_alt = &pointer_struct->sensorDataPointer->barometer_data.altitude;
    mutex_lowG_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG;
    mutex_highG_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_highG;
    dataMutex_barometer_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_barometer;
    dataMutex_state_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_state;
    stateData_ = &pointer_struct->stateData;

    std::ofstream Kalman;
    Kalman.open("Kalman.csv");
    Kalman << "Pos,Vel,Accel,invertboi_00,invertboi_01,invertboi_10,invertboi_11\n";
    Kalman.close();

    std::ofstream extension;
    extension.open("extension.csv");
    extension << "Flap Extension (m)\n";
    extension.close(); 
}

void KalmanFilter::kfTickFunction() {
    priori();
    update();
}

void KalmanFilter::Initialize(float pos_f, float vel_f, float accel_f) {
    // set x_k
    x_k(0,0) = pos_f;
    x_k(1,0) = vel_f;
    x_k(2,0) = accel_f;
    
    // set F
    F_mat(0, 1) = s_dt;
    F_mat(0, 2) = (s_dt*s_dt) / 2;
    F_mat(1, 2) = s_dt;

    F_mat(0, 0) = 1;
    F_mat(1, 1) = 1;
    F_mat(2, 2) = 1;

    // set H
    H(0,0) = 1;
    H(1,2) = 1;

    // set P_k
    P_k(0,0) = .058;
    P_k(0,1) = .024;
    P_k(0,2) = .004;
    P_k(1,1) = .019;
    P_k(2,2) = .0086;
    P_k(1,2) = .007;
    P_k(2,1) = P_k(1,2);
    P_k(1,0) = P_k(0,1);
    P_k(2,0) = P_k(0,2);
    P_k(1,0) = P_k(0,1);

    // set Q
    Q(0,0) = pow(s_dt,5) / 20;
    Q(0,1) = (pow(s_dt,4) / 8);
    Q(0,2) = pow(s_dt,3) / 6;
    Q(1,1) = pow(s_dt,3) / 8;
    Q(1,2) = pow(s_dt,2) / 2;
    Q(2,2) = s_dt;
    Q(1,0) = Q(0,1);
    Q(2,0) = Q(0,2);
    Q(2,1) = Q(1,2);

    // float scale_fact = 22.19;
    // float scale_fact = 13.25;
    // float scale_fact = .00899;
    float scale_fact = 12.;
    Q = Q * scale_fact;

    // set R
    R(0,0) = 2;
    R(1,1) = 0.01;

    // set B
    B(2,0) = -1;
}

void KalmanFilter::Initialize(float pos_f, float vel_f) {
    // set x_k
    x_k(0,0) = pos_f;
    x_k(1,0) = vel_f;
    
    // set F
    F_mat(0, 1) = s_dt;

    F_mat(0, 0) = 1;
    F_mat(1, 1) = 1;

    // set H
    H(0,0) = 1;

    // set R
    R(0,0) = 12;

    // set B
    B(2,0) = -1;
}

void KalmanFilter::priori() {
    // x_priori = (F @ x_k) + ((B @ u).T) #* For some reason doesnt work when B or u is = 0
    x_priori = (F_mat * x_k);
    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
}

void KalmanFilter::update() {
    // Update Kalman Gain
    temp = (((H * P_priori * H.transpose()) + R).inverse());
    K = (P_priori * H.transpose()) * temp;

    // Sensor Measurements
    chMtxLock(mutex_highG_);
    y_k(1,0) = (*gz_H) * 9.81;
    // Serial.println("HIGH G ACCEL Z: ");
    // Serial.println(std::to_string((*gz_H)*9.81).c_str());
    chMtxUnlock(mutex_highG_);

    chMtxLock(dataMutex_barometer_);
    y_k(0,0) = *b_alt;
    // std::cout<< y_k(0,0) <<std::endl;
    chMtxUnlock(dataMutex_barometer_);
    
    // # Posteriori Update
    x_k = x_priori + K * (y_k - (H * x_priori));
    P_k = (identity - K*H) * P_priori;

    // check overflow on Kalman gain
    // for(int i = 0; i < 3; i++) {
    //     for(int j = 0; j < 3; j++) {
    //         if(P_k(i,j) < 1e-7 && P_k(i,j) > -1e-7)
    //             if (P_k(i,j) > 0)
    //                 P_k(i, j) = 1e-7;
    //             if (P_k(i,j) < 0)
    //                 P_k(i, j) = -1e-7;
    //     }
    // }

    // std::cout << "STATE ESTIMATION: " << std::endl;
    // std::cout << x_k(0,0) << " m" << std::endl;
    // std::cout << x_k(1,0) << " m/s" << std::endl;
    // std::cout << x_k(2,0) << " m/s^2" << std::endl;

    // if (x_k(0,0) > 9200) {
    //     std::cout << x_k(0,0) << std::endl;
    // }

    chMtxLock(dataMutex_state_);
    stateData_->state_x = x_k(0,0);
    stateData_->state_vx = x_k(1,0);
    stateData_->state_ax = x_k(2,0);
    chMtxUnlock(dataMutex_state_);

    std::string str = std::to_string(x_k(0,0)) + "," + std::to_string(x_k(1,0)) + "," + std::to_string(x_k(2,0)) + "," + std::to_string(((H * P_priori * H.transpose()) + R)(0,0)) + 
    "," + std::to_string(((H * P_priori * H.transpose()) + R)(0,1)) + "," + std::to_string(((H * P_priori * H.transpose()) + R)(1,0)) + "," + std::to_string(((H * P_priori * H.transpose()) + R)(1,1)) + '\n';

    std::ofstream Kalman;
    Kalman.open("Kalman.csv", std::ios::app);
    Kalman << str;
    Kalman.close();
}
