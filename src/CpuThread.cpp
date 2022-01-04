/**
 * @file        CpuThread.h
 * @authors     Ayberk Yaraneri, Brooke Novosad, Nicholas Phillips
 *
 * @brief       Member function implementations for threads on the rocket
 * classes
 *
 * The CpuThread holds the function of the thread from the flight software.
 * The tick functions represent the tick of the rocket and the specific time
 * to run for each thread.
 *
 */
#include "CpuThread.h"
double Rocket_FSM::real_tick(CpuStateContext& context) {
    (void)context;
    return 1.0;
}
