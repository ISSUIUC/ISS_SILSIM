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

#include <fstream>
#include <memory>
#include <string>

#include "Atmosphere/Atmosphere.h"
#include "Physics/PhysicsEngine.h"
#include "Propulsion/Propulsion.h"
#include "Rocket/Rocket.h"
#include "Sensors/Sensor.h"

class Simulation {
   public:
    Simulation(double tStep, PhysicsEngine* engine, Atmosphere& atmosphere, Rocket& rocket, RocketMotor& motor);

    void init();
    bool step();  // iterators at home

    void set_PhysicsEngine(PhysicsEngine* engine) { engine_ = engine; };

    /************************* Sensor interface ***************************/
    void add_sensor(Sensor* sensor);
    void update_sensors();
    //time in seconds
    double get_time(){ return tStamp_;};

   private:
    double tStamp_;
    double tStep_;

    PhysicsEngine* engine_;

    Atmosphere& atmoshpere_;

    Rocket& rocket_;

    RocketMotor& motor_;

    // Sensors
    std::vector<Sensor*> sensors_;  // array of sensors on the rocket

    int state = 0;
};
