/**
 * @file        CpuState.h
 * @authors     Ayberk Yaraneri, Brooke Novosad, Nicholas Phillips
 *
 * @brief       Class definition for CpuState threads
 * classes
 *
 * The CpuState holds the tick function for the threads to run on. It also
 * contains the vector of different threads and its runtimes in absolute time
 * so we know when to run each thread and they are independent of each other.
 *
 */
#ifndef SILSIM_CPUSTATE_H
#define SILSIM_CPUSTATE_H

#include <memory>
#include <vector>

#include "CpuThread.h"
class CpuState {
   public:
    CpuState();

    void add_thread(CpuThread* thread);

    void tick(double timestamp);

   private:
    // thead, when the threads should next run in absolute time
    std::vector<std::pair<CpuThread*, double>> threads_;
};

#endif  // SILSIM_CPUSTATE_H
