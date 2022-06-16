#include <rk4.h>
#include <PWMServo.h>
#include "dataLog.h"
#include "ServoControl.h"
#include "Eigen30.h"
#include "Eigen/Core"
#include <math.h>
#include <array>

#include "GlobalVars.h"

using std::array;

class Controller {
    public:
    void ctrlTickFunction();
    bool ActiveControl_ON();
    Controller(struct pointers* pointer_struct, PWMServo* twisty_boi);
    void setLaunchPadElevation();
    
    PWMServo* twisty_boi_;
    mutex_t* dataMutex_state_;
    struct StateData* stateData_;
    rk4 rk4_;
    float kp = 0.000024075;
    float apogee_des = 4572;
    float min_extension = 0;
    float max_extension = 17.88 / 1000;
    float dt = .006;
    float du_max = 0.01;
    float flap_width = 35.1 / 1000; // m
    float launch_pad_alt = 0;
    const float apogee_des_agl = 9144;
    float apogee_des_msl = apogee_des_agl;
    float* b_alt;
    mutex_t* dataMutex_barometer_;
    FSM_State* current_state;
    ServoControl activeControlServos;

    float apogee_est_ = 0.0;
    float u_ = 0.0;
    float prev_u_ = 0.0;

    // SILSIM Data Logging
    void log_controller_state(double tStamp);
    std::shared_ptr<spdlog::logger> controller_logger_;
    std::string datalog_format_string_ = 
        "timestamp,apogee_est,u";
};