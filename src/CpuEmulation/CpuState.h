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

#include <fstream>
#include <memory>
#include <vector>

#include "CpuThread.h"

class CpuState {
   public:
    CpuState(Accelerometer* accelerometer, Thermometer* thermometer,
             Barometer* barometer, Gyroscope* gyroscope, GPSSensor* gps,
             Magnetometer* mag, Flaps* flaps, std::ostream* telemetry)
        : context() {
        context.accelerometer_pointer = accelerometer;
        context.barometer_pointer = barometer;
        context.gyroscope_pointer = gyroscope;
        context.gps_pointer = gps;
        context.thermometer_pointer = thermometer;
        context.magnetometer_pointer = mag;
        context.add_thread = [&](void* thd_class) {
            add_thread(
                std::unique_ptr<CpuThread>(static_cast<CpuThread*>(thd_class)));
        };
        context.flaps = flaps;
        context.telemetry_log = telemetry;
    }

    void add_thread(std::unique_ptr<CpuThread> thread);
    void tick(double timestamp);
    std::unordered_map<std::string, FileStorage> const& get_filesystem() const {
        return context.SD.map;
    }

   private:
    CpuStateContext context;
    std::vector<std::pair<std::unique_ptr<CpuThread>, double>> threads_;
    // thead, when the threads_ should next run in absolute time
    bool has_started{};
};

#endif  // SILSIM_CPUSTATE_H
