#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <iostream>

#include "CpuState.h"
#include "Rocket.h"
#include "Sensor.h"
#include "Simulation.h"

constexpr double deg2rad = 3.14159265 / 180.0;

int main() {
    spdlog::set_level(spdlog::level::debug);
    // comment below is used if we want to change the format of the logging
    // spdlog::set_pattern("*** [%H:%M:%S %z] [thread %t] %v ***");

    Rocket rocket;

    double mass = rocket.get_structural_mass();
    std::array<double, 9> I_tensor{};
    I_tensor[0] = (1.0 / 12.0) * mass * 5.182 * 5.182 * 25;
    I_tensor[4] = I_tensor[0];
    I_tensor[8] = 0.5 * mass * 0.0762 * 0.0762;
    rocket.set_I(I_tensor);

    double angle = 5.0 * deg2rad;
    Quaterniond start_ornt{cos(angle / 2.0), sin(angle / 2.0) * 0.707,
                           sin(angle / 2.0) * 0.707, 0};
    rocket.set_q_ornt(start_ornt);

    // Construct some sensors
    Accelerometer accel1("LSM9_accel", rocket, 100);
    accel1.enable_noise_injection();
    Gyroscope gyro1("LSM9_gyro", rocket, 100);

    // 3.5 second burn time @ 4000 Newton constant thrust (L ish motor I think)
    // ConstantThrustSolidMotor motor(3.594, 5772.1);

    // Cesaroni N5800 Motor
    ThrustCurveSolidMotor motor("thrust_curves/cesaroni_n5800.csv");

    // HELLO, RUNGEKUTTA?
    // RUNGEKUTTA BROKE
    // RungeKutta engine(rocket, motor);
    // UNDERSTANDABLE. HAVE A GREAT DAY
    ForwardEuler engine(rocket, motor);

    CpuState cpu;

    Simulation sim(0.01, &engine, rocket, motor, cpu, "sim_data/data.csv");

    sim.add_sensor(&accel1);
    // sim.add_sensor(&gyro1);

    std::cout << "Running Sim!" << std::endl;

    // run 10000 steps
    sim.run(10000);

    return 0;
}
