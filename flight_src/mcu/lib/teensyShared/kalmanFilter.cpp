#include "kalmanFilter.h"
#define EIGEN_MATRIX_PLUGIN "MatrixAddons.h"

KalmanFilter::KalmanFilter(struct pointers* pointer_struct) {
    gy_L = &pointer_struct->sensorDataPointer->lowG_data.gy;
    gy_H = &pointer_struct->sensorDataPointer->highG_data.hg_ay;
    b_alt = &pointer_struct->sensorDataPointer->barometer_data.altitude;
    mutex_lowG_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG;
    mutex_highG_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_highG;
    dataMutex_barometer_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_barometer;
    dataMutex_state_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_state;
    stateData_ = &pointer_struct->stateData;
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
    F_mat(0, 2) = s_dt;

    // set H
    H(0,0) = 1;
    H(1,2) = 1;

    // set P_k
    P_k(0,0) = .018;
    P_k(0,1) = .009;
    P_k(0,2) = .005;
    P_k(1,1) = .009;
    P_k(2,2) = 10;
    P_k(1,3) = .0045;
    P_k(3,1) = P_k(1,3);
    P_k(1,0) = P_k(0,1);
    P_k(2,0) = P_k(0,2);
    P_k(1,0) = P_k(0,1);

    // set Q
    

    // set R
    R(0,0) = 12;
    R(1,1) = 1.4;
}

void KalmanFilter::priori() {
    // x_priori = (F @ x_k) + ((B @ u).T) #* For some reason doesnt work when B or u is = 0
    x_priori = F_mat * x_k;
    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
}

void KalmanFilter::update() {
    // Update Kalman Gain
    temp = (((H * P_priori * H.transpose()) + R).array().inverse());
    K = (P_priori * H.transpose()) * temp;

    // Sensor Measurements
    chMtxLock(mutex_lowG_);
    y_k(0,0) = *gy_H;
    chMtxUnlock(mutex_lowG_);

    chMtxLock(dataMutex_barometer_);
    y_k(1,0) = *b_alt;
    chMtxUnlock(dataMutex_barometer_);
    
    // # Posteriori Update
    x_k = x_priori + K * (y_k - (H * x_priori));
    P_k = (identity - K*H) * P_priori;

    chMtxLock(dataMutex_state_);
    stateData_->state_x = x_k(0,0);
    stateData_->state_vx = x_k(1,0);
    stateData_->state_ax = x_k(2,0);
    chMtxUnlock(dataMutex_state_);
}