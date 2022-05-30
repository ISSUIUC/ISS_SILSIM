#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <iostream>

#include "CpuState.h"
#include "RASAeroImport.h"
#include "Rocket.h"
#include "Sensor.h"
#include "Simulation.h"

// Shortening the typename for   a e s t h e t i c s
typedef std::shared_ptr<spdlog::sinks::basic_file_sink_mt>
    spdlog_basic_sink_ptr;

/****************** Conversion Constants  ******************/
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
    spdlog_basic_sink_ptr silsim_datalog_sink =
        std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/hmmm.log");

    silsim_datalog_sink->set_level(spdlog::level::info);

    // comment below is used if we want to change the format of the logging
    silsim_datalog_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%t] [%l] [%n] [%v]");

    std::shared_ptr<RASAeroImport> rasaero_import =
        std::make_shared<RASAeroImport>(
            silsim_datalog_sink,
            "utils/RASAero_fetch/output/RASAero_Intrepid_5800_mk6.csv");

    Rocket rocket(silsim_datalog_sink, rasaero_import);

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
    Accelerometer accel1("LSM9_accel", rocket, 100, silsim_datalog_sink);
    accel1.enable_noise_injection();
    Gyroscope gyro1("LSM9_gyro", rocket, 100, silsim_datalog_sink);

    // Modeling Cesaroni N5800, 3.49s burn, 5800N avg thrust, 9.021kg prop
    // weight
    // ConstantThrustSolidMotor motor(3.49, 5800.0, 9.021, silsim_sink);

    // Cesaroni N5800 Motor
    ThrustCurveSolidMotor motor("thrust_curves/cesaroni_n5800.csv", 9.425,
                                silsim_datalog_sink);

    // RungeKutta engine(rocket, motor, silsim_datalog_sink);
    ForwardEuler engine(rocket, motor, silsim_datalog_sink);

    CpuState cpu;

    Simulation sim(0.01, &engine, rocket, motor, cpu, silsim_datalog_sink);

    sim.add_sensor(&accel1);
    sim.add_sensor(&gyro1);

    std::cout << "Running Sim!" << std::endl;

    // run 10000 steps
    sim.run(10000);

    return 0;
}
