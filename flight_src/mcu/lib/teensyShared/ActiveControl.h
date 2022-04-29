#include<rk4.h>
#include <PWMServo.h>
#include "dataLog.h"
#include "ServoControl.h"

class Controller {
    public:
    void ctrlTickFunction();
    bool ActiveControl_ON();
    Controller(struct pointers* pointer_struct, PWMServo* twisty_boi);

    PWMServo* twisty_boi_;
    mutex_t* dataMutex_state_;
    struct StateData* stateData_;
    rk4 rk4_;
    float kp = 0.00008;
    float apogee_des = 11000;
    float min_extension = 0;
    float max_extension = 20.7264 / 1000;
    float dt = .006;
    float prev_u = 0;
    float du_max = 0.001;
    FSM_State* current_state;
    ServoControl activeControlServos;
};