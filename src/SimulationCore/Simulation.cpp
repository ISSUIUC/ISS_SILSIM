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

#include <spdlog/logger.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>

#include "Atmosphere.h"
#include "CpuState.h"
#include "PhysicsEngine.h"
#include "Propulsion.h"
#include "Rocket.h"
#include "Sensor.h"

#define RAD2DEG (180.0 / 3.14159265)

using Eigen::Vector3d;

Simulation::Simulation(double tStep, PhysicsEngine* engine,
                       Atmosphere& atmosphere, Rocket& rocket,
                       RocketMotor& motor, CpuState& cpu,
                       spdlog_basic_sink_ptr silsim_sink)
    : tStamp_(0),
      tStep_(tStep),
      engine_(engine),
      atmoshpere_(atmosphere),
      rocket_(rocket),
      motor_(motor),
      cpu_(cpu) {
    if (silsim_sink) {
        sim_logger_ =
            std::make_shared<spdlog::logger>("Simulation", silsim_sink);
        sim_logger_->info("DATALOG_FORMAT," + datalog_format_string);
        sim_logger_->set_level(spdlog::level::debug);
    }
}

void Simulation::run(int steps) {
    // Update rocket's initial aerodynamic coefficients
    rocket_.update_aero_coefficients(motor_.is_burning(tStamp_));

    // Initial update of total mass to include propellant mass
    double rocket_structural_mass = rocket_.get_structural_mass();
    rocket_.set_total_mass(rocket_structural_mass +
                           motor_.get_propellant_mass(tStamp_));

    update_sensors();

    // An initial data log at timestamp 0.0
    log_simulation_state();
    log_simulation_debug();
    log_sensors();
    rocket_.log_rocket_state(tStamp_);
    rocket_.log_control_surfaces(tStamp_);
    motor_.log_motor_state(tStamp_);
    atmoshpere_.log_atmosphere_state(tStamp_);

    // Wait 15 seconds before ignition
    while (tStamp_ < 40.0) {
        // Update all sensors' internal state
        update_sensors();

        // Tick the emulated CPU and threads
        cpu_.tick(tStamp_);

        // Do all the data logging!
        log_simulation_state();
        log_simulation_debug();
        log_sensors();
        rocket_.log_rocket_state(tStamp_);
        rocket_.log_control_surfaces(tStamp_);
        motor_.log_motor_state(tStamp_);

        tStamp_ += tStep_;
    }

    // send it
    motor_.ignite(tStamp_);

    for (int iter = 0; iter < steps; ++iter) {
        // Get ENU frame rocket state
        Vector3d r_vect_enu = rocket_.get_r_vect();
        Vector3d r_dot_enu = rocket_.get_r_dot();

        // Update rocket's control surfaces
        rocket_.update_flaps(tStep_);

        // Update rocket's aerodynamic coefficients for current state
        rocket_.update_aero_coefficients(motor_.is_burning(tStamp_));

        // Update total mass to include new propellant mass
        rocket_.set_total_mass(rocket_structural_mass +
                               motor_.get_propellant_mass(tStamp_));

        // Perform physics magic to march simulation forward in time
        engine_->march_step(tStamp_, tStep_);

        // Tick the emulated CPU and threads
        cpu_.tick(tStamp_);

        // Move time forward by a timestep
        tStamp_ += tStep_;

        // Update all sensors' internal state
        update_sensors();

        // Do all the data logging!
        log_simulation_state();
        log_simulation_debug();
        log_sensors();
        rocket_.log_rocket_state(tStamp_);
        rocket_.log_control_surfaces(tStamp_);
        motor_.log_motor_state(tStamp_);

        // End simulation if apogee is reached
        if (r_dot_enu.z() < -3.0) {
            log_simulation_event("Apogee Reached. Halting Simulation.");
            break;
        }
    }
}

/**
 * @brief Adds a new sensor on the rocket to the simulation
 *
 * @param sensor A pointer to the new Sensor object to be added
 */
void Simulation::add_sensor(Sensor* sensor) { sensors_.push_back(sensor); }

/**
 * @brief Updates all sensors' internal data
 *
 */
void Simulation::update_sensors() {
    for (Sensor* sensor : sensors_) {
        sensor->update_data(tStamp_);
    }
}

/**
 * @brief Calls the data logging functions of all sensors
 *
 */
void Simulation::log_sensors() {
    for (Sensor* sensor : sensors_) {
        sensor->log_sensor_state(tStamp_);
    }
}

/*****************************************************************************/
/*                            LOGGING FUNCTIONS                              */
/*****************************************************************************/

void Simulation::log_simulation_state() {
    if (sim_logger_) {
        Vector3d r_dot_rf = rocket_.enu2r(rocket_.get_r_dot());
        Quaterniond q = rocket_.get_q_ornt();

        // eqns from
        // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        double yaw = atan2(2.0 * (q.w() * q.x() + q.z() * q.y()),
                           1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())) *
                     RAD2DEG;
        double pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x())) * RAD2DEG;
        double roll = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                            -1.0 + 2.0 * (q.w() * q.w() + q.x() * q.x())) *
                      RAD2DEG;

        double alpha = acos(r_dot_rf.z() / r_dot_rf.norm());

        Vector3d rocket_axis(0, 0, 1);
        rocket_axis = rocket_.r2enu(rocket_axis);

        // Get Atmosphere state
        Vector3d pos_ecef = rocket_.position_enu2ecef(rocket_.get_r_vect());
        Vector3d geod = rocket_.ecef2geod(pos_ecef);
        double altitude = geod.z();
        double temperature = Atmosphere::get_temperature(altitude);
        double pressure = Atmosphere::get_pressure(altitude);
        double density = Atmosphere::get_density(altitude);
        double speed_of_sound = Atmosphere::get_speed_of_sound(altitude);

        std::stringstream datalog_ss;

        // clang-format off
        datalog_ss << "DATA,";

        datalog_ss << tStamp_ << ",";

        datalog_ss << temperature << ","
                   << pressure << ","
                   << density << ","
                   << speed_of_sound << ",";

        datalog_ss << roll << ","
                   << pitch << ","
                   << yaw << ",";

        datalog_ss << rocket_axis.x() << ","
                   << rocket_axis.y() << ","
                   << rocket_axis.z();
        // clang-format on

        sim_logger_->info(datalog_ss.str());
    }
}

void Simulation::log_simulation_event(std::string message) {
    if (sim_logger_) {
        std::stringstream datalog_ss;

        datalog_ss << "EVENT," << tStamp_ << "," << message;

        sim_logger_->info(datalog_ss.str());
    }
}

void Simulation::log_simulation_debug() {
    if (sim_logger_) {
        // Get ENU frame rocket state
        Vector3d r_vect_enu = rocket_.get_r_vect();
        Vector3d r_dot_enu = rocket_.get_r_dot();
        Quaterniond q = rocket_.get_q_ornt();

        // Get rocket frame state
        Vector3d r_dot_rf = rocket_.enu2r(rocket_.get_r_dot());
        Vector3d r_ddot_rf = rocket_.enu2r(rocket_.get_r_ddot());
        Vector3d f_net_rf = rocket_.enu2r(rocket_.get_f_net());
        Vector3d w_net_rf = rocket_.enu2r(rocket_.get_w_vect());
        Vector3d m_net_rf = rocket_.enu2r(rocket_.get_m_net());

        // eqns from
        // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        double yaw = atan2(2.0 * (q.w() * q.x() + q.z() * q.y()),
                           1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())) *
                     RAD2DEG;
        double pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x())) * RAD2DEG;
        double roll = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                            -1.0 + 2.0 * (q.w() * q.w() + q.x() * q.x())) *
                      RAD2DEG;

        double alpha = acos(r_dot_rf.z() / r_dot_rf.norm());
        sim_logger_->debug("Timestamp: {}", tStamp_);
        sim_logger_->debug("ENU Frame R-Vector: <{}, {}, {}>", r_vect_enu.x(),
                           r_vect_enu.y(), r_vect_enu.z());
        sim_logger_->debug("Rocket Frame Velocity: <{}, {}, {}>", r_dot_rf.x(),
                           r_dot_rf.y(), r_dot_rf.z());
        sim_logger_->debug("Rocket Frame Accel: <{}, {}, {}>", r_ddot_rf.x(),
                           r_ddot_rf.y(), r_ddot_rf.z());
        sim_logger_->debug("Rocket Frame F-Net: <{}, {}, {}>", f_net_rf.x(),
                           f_net_rf.y(), f_net_rf.z());
        sim_logger_->debug("Rocket Frame W-Net: <{}, {}, {}>", w_net_rf.x(),
                           w_net_rf.y(), w_net_rf.z());
        sim_logger_->debug("Rocket Frame T-Net: <{}, {}, {}>", m_net_rf.x(),
                           m_net_rf.y(), m_net_rf.z());
        sim_logger_->debug("ROLL: {} PITCH: {} YAW: {}  [deg]", roll, pitch,
                           yaw);
        sim_logger_->debug("alphaSIM: {}  [deg]", alpha * RAD2DEG);
    }
}
