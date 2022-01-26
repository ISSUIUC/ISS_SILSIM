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
    // returns sleep time
    double tick(CpuStateContext& context) {
        // some checks
        return loop();
    }
    virtual ~CpuThread() = default;

   private:
    virtual void setup(void* args) = 0;
    // returns sleep time
    virtual double loop() = 0;
};

#endif  // SILSIM_CPUTHREAD_H
