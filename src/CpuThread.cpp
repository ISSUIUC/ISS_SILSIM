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

CpuThread::CpuThread(void (*FSW_function)()) { FSW_function_ = FSW_function; }

/**
 * @brief Returns the sleep time of the thread
 *
 * @param context The current time of the rocket
 */

double CpuThread::tick(CpuStateContext const& context) {
    return real_tick_(context);
}

/**
 * @brief Calls the function of the thread and returns the time in absolute
 *
 * @param context The current time of the rocket
 */

double CpuThread::real_tick_(CpuStateContext const& context) {
    (*FSW_function_)();

    return 0.5f;
}
