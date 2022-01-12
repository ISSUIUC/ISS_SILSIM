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

#include <cmath>
#include <iostream>
#include <string>

#include "CpuState.h"
#include "PhysicsEngine.h"
#include "Propulsion.h"
#include "Rocket.h"
#include "Sensor.h"
#include <Eigen/Dense>
#include "quaternion.h"

#define RAD2DEG (180.0 / 3.14159265)

using Eigen::Vector3d;


Simulation::Simulation(double tStep, PhysicsEngine* engine, Rocket& rocket,
                       SolidMotor& motor, CpuState& cpu, std::string filename
                       // std::vector<Sensor&>& sensors
                       )
    : tStamp_(0),
      tStep_(tStep),
      engine_(engine),
      rocket_(rocket),
      motor_(motor),
      cpu_(cpu),
      filename_(filename) {
    sim_log =
        spdlog::basic_logger_mt("Simulation_Logger", "logs/simulation.log");
}

void Simulation::run(int steps) {
    std::ofstream dataFile(filename_);

    Vector3d r_vect;
    Vector3d r_dot;
    Vector3d r_ddot;
    Vector3d f_net;
    Vector3d w_net;

    Quaternion<double> q_ornt;

    double roll, pitch, yaw;

    motor_.ignite(tStamp_);
    for (int iter = 0; iter < steps; ++iter) {
        rocket_.get_r_vect(r_vect);
        rocket_.get_r_dot(r_dot);
        rocket_.get_r_ddot(r_ddot);
        rocket_.get_f_net(f_net);
        rocket_.get_w_vect(w_net);
        rocket_.get_q_ornt(q_ornt);

        double s = q_ornt.Gets();
        double x = q_ornt.Getx();
        double y = q_ornt.Gety();
        double z = q_ornt.Getz();

        // eqns from
        // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        yaw =
            atan2(2.0 * (s * x + z * y), 1.0 - 2.0 * (x * x + y * y)) * RAD2DEG;
        pitch = asin(2.0 * (s * y - z * x)) * RAD2DEG;
        roll = atan2(2.0 * (s * z + x * y), -1.0 + 2.0 * (s * s + x * x)) *
               RAD2DEG;

        double alpha = acos(rocket_.i2r(r_dot).z() / (r_dot.norm()));
        sim_log->debug("Timestamp: {}", tStamp_);
        sim_log->debug("R-Vector: <{}, {}, {}>", r_vect.x(), r_vect.y(), r_vect.z());
        sim_log->debug("Velocity: <{}, {}, {}>", r_dot.x(), r_dot.y(), r_dot.z());
        sim_log->debug("Accel: <{}, {}, {}>", r_ddot.x(), r_ddot.y(), r_ddot.z());
        sim_log->debug("F-Net: <{}, {}, {}>", f_net.x(), f_net.y(), f_net.z());
        sim_log->debug("W-Net: <{}, {}, {}>", w_net.x(), w_net.y(), w_net.z());
        sim_log->debug("ROLL: {} PITCH: {} YAW: {}  [deg]", roll, pitch, yaw);
        sim_log->debug("alphaSIM: {}  [deg]", alpha * RAD2DEG);
        Vector3d rocket_axis(0, 0, 1);
        rocket_axis = rocket_.r2i(rocket_axis);

        engine_->march_step(tStamp_, tStep_);

        update_sensors();

        cpu_.tick(tStamp_);

        dataFile << tStamp_ << ",";
        dataFile << r_vect.x() << "," << r_vect.y() << "," << r_vect.z() << ",";
        dataFile << r_dot.x() << "," << r_dot.y() << "," << r_dot.z() << ",";
        dataFile << r_ddot.x() << "," << r_ddot.y() << "," << r_ddot.z() << ",";
        dataFile << f_net.x() << "," << f_net.y() << "," << f_net.z() << ",";
        dataFile << s << "," << x << "," << y << "," << z << ",";
        dataFile << roll << "," << pitch << "," << yaw << ",";
        dataFile << rocket_axis.x() << "," << rocket_axis.y() << ","
                 << rocket_axis.z() << ",";

        Vector3d sensor_data;
        sensors_[0]->get_data(sensor_data);
        dataFile << sensor_data.x() << "," << sensor_data.y() << ","
                 << sensor_data.z();

        dataFile << "\n";

        tStamp_ += tStep_;

        if (r_dot.z() < -3.0) {
            break;
        }
    }

    dataFile.close();
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
    for (auto sensor : sensors_) {
        sensor->update_data(tStamp_);
    }
}
