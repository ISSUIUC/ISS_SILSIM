/**
 * @file        CpuThread.h
 * @authors     Ayberk Yaraneri, Brooke Novosad, Nicholas Phillips
 *
 * @brief       Class definition for threads on the rocket
 * classes
 *
 * The CpuThread holds the function of the thread from the flight software.
 * The tick functions represent the tick of the rocket and the specific time
 * to run for each thread.
 *
 */

#ifndef SILSIM_CPUTHREAD_H
#define SILSIM_CPUTHREAD_H

#include "CpuStateContext.h"

#define THD_WORKING
//#define THD_FUNCTION(name, arg)

class CpuThread {
   public:
    CpuThread(uint8_t priority) : priority(priority) {}
    // returns sleep time
    double tick(double timestamp);
    virtual ~CpuThread() = default;

   private:
    // returns sleep time
    uint8_t priority;
    virtual double loop(double timestamp) = 0;
};

#endif  // SILSIM_CPUTHREAD_H
