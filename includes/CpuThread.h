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

struct CpuStateContext {
    double timestamp;
};

class CpuThread {
   public:
    explicit CpuThread(void (*FSW_function)());
    double tick(CpuStateContext const& context);

   private:
    void (*FSW_function_)();
    double real_tick_(CpuStateContext const& context);
};

#endif  // SILSIM_CPUTHREAD_H
