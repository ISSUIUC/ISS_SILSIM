//
// Created by 16182 on 9/30/2021.
//

#include "CpuStateContext.h"
#ifndef SILSIM_CPUTHREAD_H
#define SILSIM_CPUTHREAD_H

#define THD_FUNCTION(name, arg) void; \
    struct name : public CpuThread { double real_tick(void*); }; \
    double name::real_tick(void*arg)

class CpuThread {
   public:
    // returns sleep time
    double tick(CpuStateContext& context) {
        // some checks
        return real_tick(&context);
    }
    virtual ~CpuThread() = default;

   private:
    // returns sleep time
    virtual double real_tick(void*) = 0;
};

#endif  // SILSIM_CPUTHREAD_H
