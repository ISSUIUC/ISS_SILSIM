#include <fstream>
#include <iostream>
#include "json.hpp"

#include "Aero/RASAeroImport.h"
#include "Atmosphere/Atmosphere.h"
#include "ControlSurfaces/Flaps.h"
#include "Rocket/Rocket.h"
#include "Sensors/Sensor.h"
#include "SimulationCore/Simulation.h"

#ifdef _WIN32
#include <windows.h>

#endif

/****************** Conversion Constants  ******************/
constexpr double deg2rad = 3.14159265 / 180.0;
constexpr double kLbsToKg = 0.453592;
constexpr double kInchToMeters = 0.0254;

/***************** Intrepid MK6 Parameters *****************/
double kIntrepidDryMass = 46.52 * kLbsToKg;
double kIntrepidWetMass = 67.30 * kLbsToKg;
double kIntrepidWetCGLocation = 82.79 * kInchToMeters;
double kIntrepidDryCGLocation = 73.06 * kInchToMeters;
double kIntrepidTotalLength = 130.0 * kInchToMeters;
double kIntrepidDiameter = 4.02 * kInchToMeters;
double kIntrepidRadius = kIntrepidDiameter / 2.0;

void load_values() {
    using json = nlohmann::json;

    std::ifstream f("sample.json");
    json data = json::parse(f);
    
    kIntrepidDryMass = data["dry_mass"];
    kIntrepidWetMass = data["wet_mass"];
    kIntrepidWetCGLocation = data["wet_center_of_gravity"];
    kIntrepidDryCGLocation = data["dry_center_of_gravity"];
    kIntrepidTotalLength = data["length"];
    kIntrepidDiameter = data["diameter"];
    kIntrepidRadius = kIntrepidDiameter / 2.0;
}

Rocket createRocket() {
    load_values();
    // RASAero Setup ----------------------------------------------------------
    RASAeroImport rasaero_import = RASAeroImport(
        "src/mcu_main/ISS_SILSIM/utils/RASAero_fetch/output/"
        "RASAero_Intrepid_5800_mk6.csv");

    // Rocket Setup -----------------------------------------------------------
    Rocket rocket{};

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
    std::shared_ptr<Flaps> flaps = std::make_shared<Flaps>();
    rocket.set_flaps(flaps);

    return rocket;
}

Atmosphere createAtmosphere() {
    Atmosphere atmosphere{};
    atmosphere.set_nominal_wind_magnitude(5.0);  // ~11.18 mph
    atmosphere.toggle_wind_direction_variance(true);
    atmosphere.toggle_wind_magnitude_variance(true);
    return atmosphere;
}

ThrustCurveSolidMotor createMotor() {
    // Cesaroni N5800, 3.49s burn, 5800N avg thrust, 9.021kg prop weight
    // ConstantThrustSolidMotor motor(3.49, 5800.0, 9.021, silsim_sink);
    ThrustCurveSolidMotor motor(
        "src/mcu_main/ISS_SILSIM/thrust_curves/cesaroni_n5800.csv", 9.425);
    return motor;
}

RungeKutta createPhysics(Rocket& rocket, ThrustCurveSolidMotor& motor,
                         Atmosphere& atmosphere) {
    // Physics Engine Setup ----------------------------------------------------
    RungeKutta engine(rocket, motor, atmosphere);
    // ForwardEuler engine(rocket, motor, atmosphere, silsim_datalog_sink);
    //    std::ofstream telemetry = std::ofstream("telemetry.log",
    //    std::ios::binary);
    return engine;
}

Simulation createSimulation(Rocket& rocket, ThrustCurveSolidMotor& motor,
                            Atmosphere& atmosphere, PhysicsEngine* engine) {
    // Simulation Setup --------------------------------------------------------
    Simulation sim(0.001, engine, atmosphere, rocket, motor);

    return sim;
}
