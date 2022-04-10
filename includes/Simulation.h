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

#include <spdlog/sinks/basic_file_sink.h>

#include <fstream>
#include <memory>
#include <string>

#include "CpuState.h"
#include "PhysicsEngine.h"
#include "Propulsion.h"
#include "Rocket.h"
#include "Sensor.h"

class Simulation {
   public:
    Simulation(std::shared_ptr<spdlog::sinks::basic_file_sink_mt> sink, double tStep, PhysicsEngine* engine, Rocket& rocket,
               RocketMotor& motor, CpuState& cpu, std::string filename
               // std::vector<Sensor&>& sensors
    );

    void run(int steps);

    void set_PhysicsEngine(PhysicsEngine* engine) { engine_ = engine; };

    /************************* Sensor interface ***************************/
    void add_sensor(Sensor* sensor);
    void update_sensors();

   private:
    double tStamp_;
    double tStep_;

    PhysicsEngine* engine_;

    Rocket& rocket_;
    RocketMotor& motor_;

    CpuState& cpu_;

    // Sensors
    std::vector<Sensor*> sensors_;  // array of sensors on the rocket

    std::string filename_;

    // logger
    std::shared_ptr<spdlog::logger> sim_log;
};

#endif
