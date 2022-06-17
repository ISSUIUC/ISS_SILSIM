
#include "kalmanFilter.h"
#include "acShared.h"

#define EIGEN_MATRIX_PLUGIN "MatrixAddons.h"

#include <iostream>
#include <string>
#include <fstream>

#include <string>
#include "GlobalVars.h"

KalmanFilter::KalmanFilter(struct pointers* pointer_struct) {

    lowG_data_ptr_ = &pointer_struct->sensorDataPointer->lowG_data;
    highG_data_ptr_ = &pointer_struct->sensorDataPointer->highG_data;
    baro_data_ptr_ = &pointer_struct->sensorDataPointer->barometer_data;
    fsm_data_ptr_ = &pointer_struct->sensorDataPointer->rocketState_data;
    state_data_ptr_ = &pointer_struct->stateData;

    mutex_lowG_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG;
    mutex_highG_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_highG;
    mutex_barometer_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_barometer;
    mutex_fsm_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_RS;
    mutex_state_ = &pointer_struct->dataloggerTHDVarsPointer.dataMutex_state;

    // SILSIM Data Logging
    kf_logger_ = std::make_shared<spdlog::logger>("KalmanFilter", silsim_datalog_sink);
    kf_logger_->info("DATALOG_FORMAT," + datalog_format_string_);
}

void KalmanFilter::kfTickFunction() {

    // Fetch current Rocket FSM state
    chMtxLock(mutex_fsm_);
    FSM_State current_state = fsm_data_ptr_->rocketState;
    chMtxUnlock(mutex_fsm_);

    // Only beging attitude dead reckoning once boost is detected
    if (current_state >= STATE_BOOST) {
        attitude_dead_reckoning();
    }

    // Perform Kalman update steps
    priori();
    update();
}

void KalmanFilter::Initialize() {
    float sum = 0;
    for(int i = 0; i < 30; i++){
        chMtxLock(mutex_barometer_);
        // std::cout<<baro_data_ptr_->altitude<<std::endl;
        sum += baro_data_ptr_->altitude;
        chMtxUnlock(mutex_barometer_);
        // chThdSleepMilliseconds(100);
    }

    // set x_k
    x_k(0,0) = sum / 30;
    x_k(0,0) = 1401;
    x_k(1,0) = 0;
    x_k(2,0) = 0;
    
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
    P_k(0,0) = 0;
    P_k(0,1) = 0;
    P_k(0,2) = 0;
    P_k(1,1) = 0;
    P_k(2,2) = 0;
    P_k(1,2) = 0;
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
    float scale_fact = .00899;
    // float scale_fact = 12.;
    Q = Q * scale_fact;

    // set R
    R(0,0) = 30;
    R(1,1) = 1;

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
    // x_priori = (F @ x_k) + ((B @ u).T) 
    // For some reason doesnt work when B or u is = 0
    x_priori = (F_mat * x_k);
    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
}

void KalmanFilter::update() {
    // Update Kalman Gain
    temp = (((H * P_priori * H.transpose()) + R).inverse());
    K = (P_priori * H.transpose()) * temp;

    // Sensor Measurements
    chMtxLock(mutex_highG_);
    y_k(1,0) = (highG_data_ptr_->hg_az) * 9.81;
    chMtxUnlock(mutex_highG_);

    chMtxLock(mutex_barometer_);
    y_k(0,0) = baro_data_ptr_->altitude;
    chMtxUnlock(mutex_barometer_);
    
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

    chMtxLock(mutex_state_);
    state_data_ptr_->state_x = x_k(0,0);
    state_data_ptr_->state_vx = x_k(1,0);
    state_data_ptr_->state_ax = x_k(2,0);
    chMtxUnlock(mutex_state_);
}

void KalmanFilter::attitude_dead_reckoning() {

    // Fetch data
    chMtxLock(mutex_lowG_);
    float gx = lowG_data_ptr_->gx;
    float gy = lowG_data_ptr_->gy;
    float gz = lowG_data_ptr_->gz;
    chMtxUnlock(mutex_lowG_);

    // Apply low-pass filter on reading
    low_pass_filter_gyro(gx, gy, gz);

	// Rate of change of quaternion from gyroscope
	float qDot0 = 0.5f * (-q1 * gx_filt - q2 * gy_filt - q3 * gz_filt);
	float qDot1 = 0.5f * (q0 * gx_filt + q2 * gz_filt - q3 * gy_filt);
	float qDot2 = 0.5f * (q0 * gy_filt - q1 * gz_filt + q3 * gx_filt);
	float qDot3 = 0.5f * (q0 * gz_filt + q1 * gy_filt - q2 * gx_filt);

    // Integrate rate-of-change of quaternion
    q0 += qDot0 * s_dt;
    q1 += qDot1 * s_dt;
    q2 += qDot2 * s_dt;
    q3 += qDot3 * s_dt;

	// Normalise quaternion
	float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

    // Write quaternion back to state struct
    chMtxLock(mutex_state_);
    state_data_ptr_->state_q0 = q0;
    state_data_ptr_->state_q1 = q1;
    state_data_ptr_->state_q2 = q2;
    state_data_ptr_->state_q3 = q3;
    state_data_ptr_->timeStamp_state = chVTGetSystemTime();
    chMtxUnlock(mutex_state_);

}

void KalmanFilter::low_pass_filter_gyro(float gx, float gy, float gz) {

    constexpr float cutoff_freq = 0.005;      // Cutoff Frequency [Hz]
    constexpr float sample_period = 0.005;   // Sample period [s]

    constexpr float RC = 1.0 / (2 * 3.14159265 * cutoff_freq);
    constexpr float coeff1 = sample_period / (sample_period + RC);
    constexpr float coeff2 = RC / (sample_period + RC);
    
    // constexpr float alpha = 0.001;

    gx_filt = coeff1*gx + coeff2*gx_filt;
    gy_filt = coeff1*gy + coeff2*gy_filt;
    gz_filt = coeff1*gz + coeff2*gz_filt;
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float KalmanFilter::invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void KalmanFilter::log_kf_state(double tStamp) {
    if (kf_logger_) {
        std::stringstream datalog_ss;

        datalog_ss << "DATA,"
                   << tStamp << ","

                   << gx_filt << ","
                   << gy_filt << ","
                   << gz_filt << ","

                   << q0 << ","
                   << q1 << ","
                   << q2 << ","
                   << q3 << ","

                   << x_k(0,0) << ","
                   << x_k(1,0) << ","
                   << x_k(2,0) << ","

                   << ((H * P_priori * H.transpose()) + R)(0,0) << ","
                   << ((H * P_priori * H.transpose()) + R)(0,1) << ","
                   << ((H * P_priori * H.transpose()) + R)(1,0) << ","
                   << ((H * P_priori * H.transpose()) + R)(1,1); 

        kf_logger_->info(datalog_ss.str());
    } 
}
