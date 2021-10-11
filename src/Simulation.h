/**
 * @file        Simulation.h
 * @authors     Ayberk Yaraneri
 *
 * @brief       Simulation class definition
 *
 * The Simulation class encapsulates meta information about the simulation.
 * The class contains references to relevant components of the simulation to
 * be performed, like a PhysicsEngine object and a Rocket object. It also
 * incorporates mechanisms for data logging and diagnostics/debugging.
 *
 * In the hierearchy of SILSIM, the Simulation class sits at the top of the
 * tree and "governs" the simulation itself.
 *
 */

#pragma once

#ifndef _SIMULATION_H_
#define _SIMULATION_H_

#include <fstream>
#include <string>

#include "CpuState.h"
#include "PhysicsEngine.h"
#include "Propulsion.h"
#include "Rocket.h"
#include "Sensor.h"
#include "quaternion.h"

class Simulation {
   public:
    Simulation(double tStep, PhysicsEngine& engine, Rocket& rocket,
               SolidMotor& motor, CpuState& cpu, std::string filename
               // std::vector<Sensor&>& sensors
               )
        : tStamp_(0),
          tStep_(tStep),
          engine_(engine),
          rocket_(rocket),
          motor_(motor),
          cpu_(cpu),
          filename_(filename){};

    void run(int steps);

    /************************* Sensor interface ***************************/
    void add_sensor(Sensor* sensor);
    void update_sensors();

   private:
    double tStamp_;
    double tStep_;

    PhysicsEngine& engine_;

    Rocket& rocket_;
    SolidMotor& motor_;

    CpuState& cpu_;

    // Sensors
    std::vector<Sensor*> sensors_;  // array of sensors on the rocket

    std::string filename_;
};

#endif
