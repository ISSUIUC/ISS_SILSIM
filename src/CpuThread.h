//
// Created by 16182 on 9/30/2021.
//

#ifndef SILSIM_CPUTHREAD_H
#define SILSIM_CPUTHREAD_H

class CpuThread {
   public:
    //returns sleep time
    double tick(){
        //some checks
        return real_tick();
    }

   private:
    //returns sleep time
    virtual double real_tick() = 0;
};

#endif  // SILSIM_CPUTHREAD_H
