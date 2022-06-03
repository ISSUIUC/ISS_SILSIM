/**
 * @file        Sensor.cpp
 * @authors     Ayberk Yaraneri
 *
 * @brief       Member function implementations for Sensor classes
 *
 * These Sensor classes encapsulate typical sensors found on Rocket avionics
 * like accelerometers and gyroscopes. This is the mechanism through which
 * the flight software being tested can obtain information of the simulated
 * rocket. The classes provide a modular a means of injecting noise and bias
 * along with other inaccuracies to make sensor measurements behave closer to
 * real hardware rather than simply providing ground truth.
 *
 */

#include "Sensor.h"

#include <Eigen/Dense>
#include <iostream>
#include <string>

using Eigen::Vector3d;

void Sensor::update_data(double tStamp) {
    (void)tStamp;
    std::cout
        << "Function 'update_data' called on Sensor base class! "
        << "This function should never be called on the base Sensor class."
        << std::endl;
}

void Sensor::get_data(Vector3d& data) {
    (void)data;
    std::cout
        << "Function 'get_data' called on Sensor base class! "
        << "This function should never be called on the base Sensor class."
        << std::endl;
}

double Sensor::get_data() {
    std::cout
        << "Function 'get_data' called on Sensor base class! "
        << "This function should never be called on the base Sensor class."
        << std::endl;
    return 0;
}

/*****************************************************************************/
/*                        GYROSCOPE MEMBER FUNCTIONS                         */
/*****************************************************************************/

/**
 * @brief Constructor for the Gryoscope class
 *
 * Class represents a MEMS Gyroscope sensor that measures instantaneous angular
 * velocity in the sensor's local frame
 *
 * @param name The name of the particular sensor being modeled
 * @param Rocket The Rocket object that this Sensor is associated with
 * @param refresh_rate The rate at which the sensor should take measurements
 * during the simulation in Hz
 * @param noise_mean The mean of the random noise generator (if noise injection
 * enabled)
 * @param noise_stddev The standard deviation of the random noise generator (if
 * noise injection enabled)
 */
Gyroscope::Gyroscope(std::string name, Rocket& rocket, double refresh_rate,
                     double noise_mean, double noise_stddev)
    : Sensor(name, rocket, refresh_rate, noise_mean, noise_stddev) {
    data_ = {0, 0, 0};
    noise_ = {0, 0, 0};
    bias_ = {0, 0, 0};
}

/**
 * @brief Updates the internal state of the Sensor. Takes a measurement if
 * needed.
 *
 * Evaluates whether or not a measurement should be taken depending on the
 * sensor's refresh rate and the time since the previous update was performed.
 * Also injects random noise into the sensor output if enabled.
 *
 * @param tStamp Current simulation timestamp
 */
void Gyroscope::update_data(double tStamp) {
    if ((tStamp - last_update_tStamp_) >= (1 / refresh_rate_)) {
        data_ = rocket_.enu2r(rocket_.get_w_vect());
        new_data_ = true;

        if (inject_noise_) {
            noise_ = randomize_vector(generator_, normal_dist_);
            data_ += noise_;
        }

        if (inject_bias_) {
            data_ += bias_;
        }
    }
}

/**
 * @brief Obtain data from the Sensor
 *
 * @param Data object to overwrite with Sensor's data
 */
void Gyroscope::get_data(Vector3d& data) {
    data = data_;
    new_data_ = false;
}

/*****************************************************************************/
/*                      ACCELEROMETER MEMBER FUNCTIONS                       */
/*****************************************************************************/

Accelerometer::Accelerometer(std::string name, Rocket& rocket,
                             double refresh_rate, double noise_mean,
                             double noise_stddev)
    : Sensor(name, rocket, refresh_rate, noise_mean, noise_stddev) {
    data_ = {0, 0, 0};
    noise_ = {0, 0, 0};
    bias_ = {0, 0, 0};
}

void Accelerometer::update_data(double tStamp) {
    if ((tStamp - last_update_tStamp_) >= (1 / refresh_rate_)) {
        // Subtract the gravity vector from the rocket's total acceleration to
        // yield specific force
        Vector3d total_accel = rocket_.enu2r(rocket_.get_r_ddot());
        Vector3d gravity_rocket_frame = rocket_.gravity_vector_rf() * 9.81;
        Vector3d specific_force = total_accel - gravity_rocket_frame;

        data_ = specific_force;
        new_data_ = true;

        if (inject_noise_) {
            noise_ = randomize_vector(generator_, normal_dist_);
            data_ += noise_;
        }

        if (inject_bias_) {
            data_ += bias_;
        }
    }
}

void Accelerometer::get_data(Vector3d& data) {
    data = data_;
    new_data_ = false;
}

/*****************************************************************************/
/*                        BAROMETER MEMBER FUNCTIONS                         */
/*****************************************************************************/

Barometer::Barometer(std::string name, Rocket& rocket, double refresh_rate,
                     double noise_mean, double noise_stddev)
    : Sensor(name, rocket, refresh_rate, noise_mean, noise_stddev) {
    data_ = rocket_.get_r_vect().x();
    bias_ = 0;
    noise_ = 0;
}

void Barometer::update_data(double tStamp) {
    if ((tStamp - last_update_tStamp_) >= (1 / refresh_rate_)) {
        data_ = rocket_.get_r_vect().x();
        new_data_ = true;

        if (inject_noise_) {
            noise_ = normal_dist_(generator_);
            data_ += noise_;
        }

        if (inject_bias_) {
            data_ += bias_;
        }
    }
}

double Barometer::get_data() {
    new_data_ = false;
    return data_;
}

Vector3d randomize_vector(std::default_random_engine& generator,
                          std::normal_distribution<double>& dist) {
    Vector3d vector;
    vector.x() = dist(generator);
    vector.y() = dist(generator);
    vector.z() = dist(generator);

    return vector;
}
