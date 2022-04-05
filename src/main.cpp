#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <iostream>

#include "CpuState.h"
#include "RASAeroImport.h"
#include "Rocket.h"
#include "Sensor.h"
#include "Simulation.h"

constexpr double deg2rad = 3.14159265 / 180.0;

constexpr double kLbsToKg = 0.453592;
constexpr double kInchToMeters = 0.0254;

/***************** Intrepid MK4 Parameters *****************/
constexpr double kIntrepidDryMass = 47.41 * kLbsToKg;
constexpr double kIntrepidWetMass = 67.30 * kLbsToKg;
constexpr double kIntrepidWetCGLocation = 82.79 * kInchToMeters;
constexpr double kIntrepidDryCGLocation = 73.06  * kInchToMeters;
constexpr double kIntrepidTotalLength = 130.0 * kInchToMeters;
constexpr double kIntrepidDiameter = 4.0 * kInchToMeters;
constexpr double kIntrepidRadius = kIntrepidDiameter / 2.0;

int main() {
    RASAeroImport rasaero_import(
        "utils/RASAero_fetch/output/RASAero_Intrepid_5800_mk6.csv");

    spdlog::set_level(spdlog::level::debug);
    // comment below is used if we want to change the format of the logging
    // spdlog::set_pattern("*** [%H:%M:%S %z] [thread %t] %v ***");

    Rocket rocket(std::make_shared<RASAeroImport>(rasaero_import));

    double mass = (((35.0 / 100.0) * kIntrepidWetMass) +
                   ((65.0 / 100.0) * kIntrepidDryMass));
    rocket.set_mass(mass);

    double nose_to_cg = (((10.0 / 100.0) * kIntrepidWetCGLocation) +
                         ((90.0 / 100.0) * kIntrepidDryCGLocation));
    rocket.set_nose_to_cg(nose_to_cg);

    std::array<double, 9> I_tensor{};
    I_tensor[0] =
        (1.0 / 12.0) * mass * kIntrepidTotalLength * kIntrepidTotalLength;
    I_tensor[4] = I_tensor[0];
    I_tensor[8] = 0.5 * mass * kIntrepidRadius * kIntrepidRadius;
    rocket.set_I(I_tensor);

    double angle = 5.0 * deg2rad;
    Quaterniond start_ornt{cos(angle / 2.0), sin(angle / 2.0) * 0.707,
                           sin(angle / 2.0) * 0.707, 0};
    rocket.set_q_ornt(start_ornt);

    // Construct some sensors
    Accelerometer accel1("LSM9_accel", rocket, 100);
    accel1.enable_noise_injection();
    Gyroscope gyro1("LSM9_gyro", rocket, 100);

    SolidMotor motor(3.49, 5800.0);

    // ForwardEuler engine(rocket, motor);
    RungeKutta engine(rocket, motor);
    CpuState cpu;

    Simulation sim(0.01, &engine, rocket, motor, cpu, "sim_data/data.csv");

    sim.add_sensor(&accel1);
    // sim.add_sensor(&gyro1);

    std::cout << "Running Sim!" << std::endl;

    // run 10000 steps
    sim.run(10000);

    return 0;
}
