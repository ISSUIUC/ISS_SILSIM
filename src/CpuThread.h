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
    CpuThread((*FSW_function)());
    //returns sleep time
    double tick(CpuStateContext const& context);

   private:
    //returns sleep time
    void (*FSW_function_)();
    double real_tick_(CpuStateContext const& context);
};

#endif  // SILSIM_CPUTHREAD_H
