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

/***************** Intrepid MK6 Parameters *****************/
constexpr double kIntrepidDryMass = 46.52 * kLbsToKg;
constexpr double kIntrepidWetMass = 67.30 * kLbsToKg;
constexpr double kIntrepidWetCGLocation = 82.79 * kInchToMeters;
constexpr double kIntrepidDryCGLocation = 73.06 * kInchToMeters;
constexpr double kIntrepidTotalLength = 130.0 * kInchToMeters;
constexpr double kIntrepidDiameter = 4.02 * kInchToMeters;
constexpr double kIntrepidRadius = kIntrepidDiameter / 2.0;

int main() {

    std::shared_ptr<spdlog::sinks::basic_file_sink_mt> silsim_sink =
        std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/hmmm.log");

    std::shared_ptr<spdlog::logger> test_logger = std::make_shared<spdlog::logger>("Test_Logger", silsim_sink);
    test_logger->log(spdlog::level::info, "hmmm test log");

    spdlog::set_level(spdlog::level::debug);
    // comment below is used if we want to change the format of the logging
    // spdlog::set_pattern("*** [%H:%M:%S %z] [thread %t] %v ***");

    std::shared_ptr<RASAeroImport> rasaero_import =
        std::make_shared<RASAeroImport>(
            silsim_sink,
            "utils/RASAero_fetch/output/RASAero_Intrepid_5800_mk6.csv");

    Rocket rocket(silsim_sink, rasaero_import);

    rocket.set_structural_mass(kIntrepidDryMass);

    double nose_to_cg = (((10.0 / 100.0) * kIntrepidWetCGLocation) +
                         ((90.0 / 100.0) * kIntrepidDryCGLocation));
    rocket.set_nose_to_cg(nose_to_cg);

    double mass = rocket.get_structural_mass();
    std::array<double, 9> I_tensor{};
    I_tensor[0] =
        (1.0 / 12.0) * mass * kIntrepidTotalLength * kIntrepidTotalLength;
    I_tensor[4] = I_tensor[0];
    I_tensor[8] = 0.5 * mass * kIntrepidRadius * kIntrepidRadius;
    rocket.set_I(I_tensor);

    double angle = 3.0 * deg2rad;
    Quaterniond start_ornt{cos(angle / 2.0), sin(angle / 2.0) * 0.707,
                           sin(angle / 2.0) * 0.707, 0};
    rocket.set_q_ornt(start_ornt);

    // Construct some sensors
    Accelerometer accel1("LSM9_accel", rocket, 100);
    accel1.enable_noise_injection();
    Gyroscope gyro1("LSM9_gyro", rocket, 100);

    // Modeling Cesaroni N5800, 3.49s burn, 5800N avg thrust, 9.021kg prop
    // weight
    // ConstantThrustSolidMotor motor(3.49, 5800.0, 9.021);

    // Cesaroni N5800 Motor
    ThrustCurveSolidMotor motor("thrust_curves/cesaroni_n5800.csv", 9.425);

    RungeKutta engine(rocket, motor);
    // ForwardEuler engine(rocket, motor);

    CpuState cpu;

    Simulation sim(silsim_sink, 0.01, &engine, rocket, motor, cpu, "sim_data/data.csv");

    sim.add_sensor(&accel1);
    // sim.add_sensor(&gyro1);

    std::cout << "Running Sim!" << std::endl;

    // run 10000 steps
    sim.run(10000);

    return 0;
}
