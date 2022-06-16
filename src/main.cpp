#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <iostream>

#include "Atmosphere.h"
#include "CpuState.h"
#include "RASAeroImport.h"
#include "Rocket.h"
#include "Sensor.h"
#include "Simulation.h"

// Shortening the typename for   a e s t h e t i c s
typedef std::shared_ptr<spdlog::sinks::basic_file_sink_mt>
    spdlog_basic_sink_ptr;

// Globally available spdlog sink
spdlog_basic_sink_ptr silsim_datalog_sink;

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
    // SILSIM Logging Setup ----------------------------------------------------
    silsim_datalog_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
        "logs/silsim_datalog.log");

    silsim_datalog_sink->set_level(spdlog::level::info);

    // comment below is used if we want to change the format of the logging
    silsim_datalog_sink->set_pattern(
        "[%Y-%m-%d %H:%M:%S.%e] [%t] [%l] [%n] [%v]");

    // RASAero Setup ----------------------------------------------------------
    std::shared_ptr<RASAeroImport> rasaero_import =
        std::make_shared<RASAeroImport>(
            silsim_datalog_sink,
            "utils/RASAero_fetch/output/RASAero_Intrepid_5800_mk6.csv");

    // Rocket Setup -----------------------------------------------------------
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

    // Contruct Control Surfaces -----------------------------------------------
    std::shared_ptr<Flaps> flaps = std::make_shared<Flaps>(silsim_datalog_sink);
    rocket.set_flaps(flaps);

    // Contruct Sensors -------------------------------------------------------
    Accelerometer accel1("LSM9_accel", rocket, 100, silsim_datalog_sink);
    accel1.enable_noise_injection();
    Gyroscope gyro1("LSM9_gyro", rocket, 100, silsim_datalog_sink, 0.001, 0.01);
    gyro1.enable_noise_injection();
    Thermometer thermo1("MS5611_thermometer", rocket, 100, silsim_datalog_sink);
    Barometer baro1("MS5611_barometer", rocket, 100, silsim_datalog_sink, 0,
                    20);
    baro1.enable_noise_injection();
    GPSSensor gps1("ZOEM8Q_gps", rocket, 10, silsim_datalog_sink);
    Magnetometer mag1("LSM9_magnetometer", rocket, 100, silsim_datalog_sink);

    // Atmosphere & Wind Setup -------------------------------------------------
    Atmosphere atmosphere(silsim_datalog_sink);
    atmosphere.set_nominal_wind_magnitude(5.0);  // ~11.18 mph
    atmosphere.toggle_wind_direction_variance(true);
    atmosphere.toggle_wind_magnitude_variance(true);

    // Motor Setup -------------------------------------------------------------
    // Cesaroni N5800, 3.49s burn, 5800N avg thrust, 9.021kg prop weight
    // ConstantThrustSolidMotor motor(3.49, 5800.0, 9.021, silsim_sink);
    ThrustCurveSolidMotor motor("thrust_curves/cesaroni_n5800.csv", 9.425,
                                silsim_datalog_sink);

    // Physics Engine Setup ----------------------------------------------------
    RungeKutta engine(rocket, motor, atmosphere, silsim_datalog_sink);
    // ForwardEuler engine(rocket, motor, atmosphere, silsim_datalog_sink);

    // CPU Emulation Setup -----------------------------------------------------
    CpuState cpu(&accel1, &thermo1, &baro1, &gyro1, &gps1, &mag1, flaps.get());

    // Simulation Setup --------------------------------------------------------
    Simulation sim(0.001, &engine, atmosphere, rocket, motor, cpu,
                   silsim_datalog_sink);

    sim.add_sensor(&accel1);
    sim.add_sensor(&gyro1);
    sim.add_sensor(&thermo1);
    sim.add_sensor(&baro1);
    sim.add_sensor(&gps1);
    sim.add_sensor(&mag1);

    // Run Simulation ----------------------------------------------------------
    std::cout << "Running Sim!" << std::endl;

    // run simulation
    sim.run(10000000);

    return 0;
}
