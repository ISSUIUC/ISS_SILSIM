//
// Created by 16182 on 9/30/2021.
//

#ifndef SILSIM_CPUSTATE_H
#define SILSIM_CPUSTATE_H

#include <memory>
#include <vector>

#include "CpuThread.h"
class CpuState {
   public:
    CpuState(Accelerometer* accelerometer, Thermometer* thermometer, Barometer* barometer, Gyroscope* gyroscope, GPSSensor* gps, Magnetometer* mag): context() {
        context.accelerometer_pointer = accelerometer;
        context.barometer_pointer = barometer;
        context.gyroscope_pointer = gyroscope;
        context.gps_pointer = gps;
        context.thermometer_pointer = thermometer;
        context.magnetometer_pointer = mag;
        context.add_thread = [&](void * thd_class){
            add_thread(std::unique_ptr<CpuThread>(static_cast<CpuThread*>(thd_class)));
        };
    }

    void add_thread(std::unique_ptr<CpuThread> thread);
    void tick(double timestamp);

   private:

    CpuStateContext context;
    std::vector<std::pair<std::unique_ptr<CpuThread>, double>> threads_;
    // thead, when the threads_ should next run in absolute time
    bool has_started{};
};

#endif  // SILSIM_CPUSTATE_H
