/**
 * @file        Simulation.cpp
 * @authors     Ayberk Yaraneri
 *
 * @brief       Simulation class member function implementations
 *
 * The Simulation class encapsulates meta information about the simulation.
 * It also contains references to relevant components of the simulation to
 * be performed, like a PhysicsEngine object and a Rocket object.
 *
 * In the hierearchy of SILSIM, the Simulation class sits at the top of the
 * tree and "governs" the simulation itself.
 *
 */

#include "Simulation.h"

#include <EigenSILSIM/Dense>
#include <cmath>
#include <iostream>
#include <string>

#define RAD2DEG (180.0 / 3.14159265)

using Eigen::Vector3d;

Simulation::Simulation(double tStep, PhysicsEngine* engine,
                       Atmosphere& atmosphere, Rocket& rocket,
                       RocketMotor& motor)
    : tStamp_(0),
      tStep_(tStep),
      engine_(engine),
      atmoshpere_(atmosphere),
      rocket_(rocket),
      motor_(motor){
}

void Simulation::init() {
    // Update rocket's initial aerodynamic coefficients
    rocket_.update_aero_coefficients(motor_.is_burning(tStamp_));

    // Initial update of total mass to include propellant mass
    double rocket_structural_mass = rocket_.get_structural_mass();
    rocket_.set_total_mass(rocket_structural_mass +
                           motor_.get_propellant_mass(tStamp_));

    update_sensors();
}

bool Simulation::step() {
    if (state == 0) {
        // Wait 15 seconds before ignition
        if (tStamp_ < 15.0) {
            // Update all sensors' internal state
            update_sensors();

            tStamp_ += tStep_;
            return true;
        } else {
            state = 1;
            return true;
        }
    } else if (state == 1) {
        motor_.ignite(tStamp_);
        state = 2;
        return true;
    } else if (state == 2) {
        // Get ENU frame rocket state
        Vector3d r_vect_enu = rocket_.get_r_vect();
        Vector3d r_dot_enu = rocket_.get_r_dot();

        // Update rocket's control surfaces
        rocket_.update_flaps(tStep_);

        // Update rocket's aerodynamic coefficients for current state
        rocket_.update_aero_coefficients(motor_.is_burning(tStamp_));

        // Update total mass to include new propellant mass
        rocket_.set_total_mass(rocket_.get_structural_mass() + motor_.get_propellant_mass(tStamp_));

        // Perform physics magic to march simulation forward in time
        engine_->march_step(tStamp_, tStep_);

        // Move time forward by a timestep
        tStamp_ += tStep_;

        // Update all sensors' internal state
        update_sensors();

        // End simulation if apogee is reached
        if (r_dot_enu.z() < -3.0) {
            state = 3;
        }
        return true;
    } else if (state == 3) {
        return false;
    }
    return false;
}

/**
 * @brief Adds a new sensor on the rocket to the simulation
 *
 * @param sensor A pointer to the new Sensor object to be added
 */
void Simulation::add_sensor(Sensor* sensor) {
    sensors_.push_back(sensor);
}

/**
 * @brief Updates all sensors' internal data
 *
 */
void Simulation::update_sensors() {
    for (Sensor* sensor : sensors_) {
        sensor->update_data(tStamp_);
    }
}