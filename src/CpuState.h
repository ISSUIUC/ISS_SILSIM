//
// Created by 16182 on 9/30/2021.
//

#ifndef SILSIM_CPUSTATE_H
#define SILSIM_CPUSTATE_H

#include "CpuThread.h"
#include<vector>
#include<memory>
class CpuState {
   public:
    CpuState(){}

    void add_thread(std::unique_ptr<CpuThread> thread);
    void tick(double timestamp);
   private:
    //thead, when the threads_ should next run in absolute time
    std::vector<std::pair<std::unique_ptr<CpuThread>, double>> threads_;
};

#endif  // SILSIM_CPUSTATE_H
