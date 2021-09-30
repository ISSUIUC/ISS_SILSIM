//
// Created by 16182 on 9/30/2021.
//

#ifndef SILSIM_CPUTHREAD_H
#define SILSIM_CPUTHREAD_H
struct CpuStateContext{
    double timestamp;
};
class CpuThread {
   public:
    //returns sleep time
    double tick(CpuStateContext const& context){
        //some checks
        return real_tick(context);
    }

   private:
    //returns sleep time
    virtual double real_tick(CpuStateContext const& context) = 0;
};

#endif  // SILSIM_CPUTHREAD_H
