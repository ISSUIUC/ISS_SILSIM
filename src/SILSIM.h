#pragma once

#include "Rocket/Rocket.h"
#include "SimulationCore/Simulation.h"

typedef struct {
    std::string file_name;
    double kRocketDryMass;
    double kRocketWetMass;
    double kRocketWetCGLocation;
    double kRocketDryCGLocation;
    double kRocketTotalLength;
    double kRocketDiameter;
    double kRocketRadius;
} RocketParameters;


Rocket createRocket(RocketParameters& params);
void configureRocket(Rocket& rocket);
Atmosphere createAtmosphere();
ThrustCurveSolidMotor createMotor();
RungeKutta createPhysics(Rocket& rocket, ThrustCurveSolidMotor& motor,
                         Atmosphere& atmosphere);
Simulation createSimulation(Rocket& rocket, ThrustCurveSolidMotor& motor,
                            Atmosphere& atmosphere, PhysicsEngine* engine);
void run_sim();