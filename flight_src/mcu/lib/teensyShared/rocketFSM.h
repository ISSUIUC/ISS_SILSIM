/**
 * @file        rocketFSM.h
 * @authors     Anshuk Chigullapali
 * 		Ayberk Yaraneri
 * 		Colin Kinsey
 *
 *
 * @brief       The implementation of the finite state machine class that
 * governs state transitions.
 *
 * The rocketFSM class encapsulates the finite state machine that dictates which
 * state the rocket is in throughout the mission. The class implements the logic
 * necessary to reliably transition between states along with hysteresis to
 * avoid premature state transitions.
 *
 * This is a highly flight critical software module and should be tested
 * throughly in simulation and on hardware targets.
 *
 */

#ifndef ROCKET_FSM_H
#define ROCKET_FSM_H

#include "GlobalVars.h"

#include "sensors.h"

class rocketFSM {
   public:
    rocketFSM(pointers *);

    void tickFSM();
    
    // SILSIM Data Logging
    void log_FSM_state(double tStamp);

   private:
    pointers *pointer_struct;

    FSM_State current_state_;

    // SILSIM Data Logging
    std::shared_ptr<spdlog::logger> fsm_logger_;
    std::string datalog_format_string = 
        "timestamp,state";
};

#endif