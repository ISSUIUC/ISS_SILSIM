#pragma once

#include "Rocket/Rocket.h"
#include "SimulationCore/Simulation.h"

Rocket createRocket();
Atmosphere createAtmosphere();
ThrustCurveSolidMotor createMotor();
RungeKutta createPhysics(Rocket& rocket, ThrustCurveSolidMotor& motor,
                         Atmosphere& atmosphere);
Simulation createSimulation(Rocket& rocket, ThrustCurveSolidMotor& motor,
                            Atmosphere& atmosphere, PhysicsEngine* engine);
void run_sim();