#include "ActiveControl.h"
#include <iostream>
#include <string>
#include <fstream>

Controller::Controller(struct pointers* pointer_struct, PWMServo* twisty_boi): activeControlServos(twisty_boi) {
    twisty_boi_ = twisty_boi;
    stateData_ = &pointer_struct->stateData;
    current_state =
        &pointer_struct->sensorDataPointer->rocketState_data.rocketState;
    // dataMutex_state_ = &pointer_struct->dataloggerTHDVarsPointer->dataMutex_state;
    
    // SILSIM Data Logging
   controller_logger_ = std::make_shared<spdlog::logger>("Controller", silsim_datalog_sink);
   controller_logger_->info("DATALOG_FORMAT," + datalog_format_string_); 
}

void Controller::ctrlTickFunction() {
    // chMtxLock(dataMutex_state_);
    array<float, 2> init = {stateData_->state_x, stateData_->state_vx};
    // chMtxUnlock(dataMutex_state_);
    apogee_est_ = rk4_.sim_apogee(init, 0.1)[0];

    u_ = kp*(apogee_est_ - apogee_des);

    float min = abs(u_ - prev_u_)/dt;

    if (du_max < min) {
        min = du_max;
    }

    float sign = 1;

    if (u_ - prev_u_ < 0) {
        sign = -1;
    }
    u_ = u_ + sign*min*dt;
    prev_u_ = u_;


    //Set flap extension limits
    if (u_ < min_extension) {
        u_ = min_extension;
    } else if (u_ > max_extension) {
        u_ = max_extension;
    }

    if (ActiveControl_ON()) {
        activeControlServos.servoActuation(u_);
    } else {
        activeControlServos.servoActuation(0);
    }

}

bool Controller::ActiveControl_ON() {

    // For some reason, this only sees IDLE through launch.

    bool active_control_on = true;
    switch (*current_state) {
        case STATE_INIT:
            // std::cout << "INIT" << std::endl;
            active_control_on = false;
            break;
        case STATE_IDLE:
            // std::cout << "IDLE" << std::endl;
            active_control_on = false;
            break;
        case STATE_LAUNCH_DETECT:
            // std::cout << "Launch Detect" << std::endl;
            active_control_on = false;
            break;
        case STATE_BOOST:
            // std::cout << "BOOST" << std::endl;
            active_control_on = false;
            break;
        case STATE_COAST:
            // std::cout << "COAST" << std::endl;
            active_control_on = true;
            break;
        case STATE_APOGEE_DETECT:
            // std::cout << "Apogee" << std::endl;
            active_control_on = false;
            break;
        default:
            active_control_on = false;
            break;
    }
    return active_control_on;
}

void Controller::log_controller_state(double tStamp) {
    if (controller_logger_) {
        std::stringstream datalog_ss;

        datalog_ss << "DATA,"
                   << tStamp << ","
                   << apogee_est_ << ","
                   << u_;

        controller_logger_->info(datalog_ss.str());
    } 
}
