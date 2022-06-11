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

extern CpuStateContext* global_context;

double CpuThread::tick(double timestamp) {
    assert(global_context != nullptr);
    // convert sleep time to seconds
    return loop(timestamp) / 1000.0;
}
