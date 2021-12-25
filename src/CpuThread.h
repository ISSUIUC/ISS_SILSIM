//
// Created by 16182 on 9/30/2021.
//

#ifndef SILSIM_CPUTHREAD_H
#define SILSIM_CPUTHREAD_H

#include "CpuStateContext.h"
class CpuThread {
   public:
    // returns sleep time
    double tick(CpuStateContext& context) {
        // some checks
        return real_tick(context);
    }
    virtual ~CpuThread() = default;

   private:
    // returns sleep time
    virtual double real_tick(CpuStateContext& context) = 0;
};

class Rocket_FSM : public CpuThread {
   public:
    Rocket_FSM();

   private:
    double real_tick(CpuStateContext& context) override;
};

#endif  // SILSIM_CPUTHREAD_H
