#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <iostream>
#include <vector>
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
    RASAeroImport rasaero_import(
        "utils/RASAero_fetch/output/RASAero_Intrepid_5800_mk6.csv");

    spdlog::set_level(spdlog::level::debug);
    // comment below is used if we want to change the format of the logging
    // spdlog::set_pattern("*** [%H:%M:%S %z] [thread %t] %v ***");

    Rocket rocket(std::make_shared<RASAeroImport>(rasaero_import));

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

    // rocket.set_r_dot(Eigen::Vector3d{0, 0, 958.75});
    // rocket.set_r_vect(Eigen::Vector3d{0, 0, 362.47128485035});

    // Construct some sensors
    Accelerometer accelerometer("LSM9_accel", rocket, 100, 0, 1.645);
    accelerometer.enable_noise_injection();
    Gyroscope gyroscope("LSM9_gyro", rocket, 100);
    Thermometer thermometer("MS5611_thermometer", rocket, 100);
    Barometer barometer("MS5611_barometer", rocket, 100);
    GPSSensor gps("ZOEM8Q_gps", rocket, 10);
    Magnetometer magnetometer("LSM9_magnetometer", rocket, 100);

    // Modeling Cesaroni N5800, 3.49s burn, 5800N avg thrust, 9.021kg prop
    // weight
    // ConstantThrustSolidMotor motor(3.49, 5800.0, 9.021);

    // Cesaroni N5800 Motor
    ThrustCurveSolidMotor motor("thrust_curves/cesaroni_n5800.csv", 9.425);

    RungeKutta engine(rocket, motor);
    CpuState cpu(&accelerometer, &thermometer, &barometer, &gyroscope, &gps, &magnetometer);

    Simulation sim(0.01, &engine, rocket, motor, cpu, "sim_data/data.csv");

    sim.add_sensor(&accelerometer);
    sim.add_sensor(&gyroscope);
    sim.add_sensor(&thermometer);
    sim.add_sensor(&barometer);
    sim.add_sensor(&gps);
    sim.add_sensor(&magnetometer);

    std::cout << "Running Sim!" << std::endl;

    // run 10000 steps
    sim.run(10000);

    auto data = cpu.get_filesystem().at("data.dat");

    auto data_dat = std::ofstream("sim_data/data_synth.dat", std::ios::binary);

    data_dat.write((const char *)data.vect.data(), data.vect.size());
    return 0;
}
