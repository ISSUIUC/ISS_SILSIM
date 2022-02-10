//
// Created by 16182 on 9/30/2021.
//

#include "CpuStateContext.h"
#ifndef SILSIM_CPUTHREAD_H
#define SILSIM_CPUTHREAD_H

#define THD_WORKING
//#define THD_FUNCTION(name, arg)

class CpuThread {
   public:
    CpuThread(uint8_t priority): priority(priority){}
    // returns sleep time
    double tick();
    virtual ~CpuThread() = default;

   private:
    // returns sleep time
    uint8_t priority;
    virtual double loop() = 0;
};

#endif  // SILSIM_CPUTHREAD_H
