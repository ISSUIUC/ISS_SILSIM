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

#include <string>

#include "Atmosphere/Atmosphere.h"

using Eigen::Vector3d;

/*****************************************************************************/
/*                        GYROSCOPE MEMBER FUNCTIONS                         */
/*****************************************************************************/

GyroscopeSensor::GyroscopeSensor(Rocket& rocket, double refresh_rate, double noise_mean, double noise_stddev)
    : Sensor(rocket, refresh_rate, noise_mean, noise_stddev) {
    data_ = {0, 0, 0};
    noise_ = {0, 0, 0};
    bias_ = {0, 0, 0};
}

void GyroscopeSensor::update_data(double tStep) {
    if ((tStep - last_update_tStep_) >= (1 / refresh_rate_)) {
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

Vector3d GyroscopeSensor::get_data() {
    new_data_ = false;
    return data_;
}

/*****************************************************************************/
/*                      ACCELEROMETER MEMBER FUNCTIONS                       */
/*****************************************************************************/

Accelerometer::Accelerometer(Rocket& rocket, double refresh_rate, double noise_mean, double noise_stddev)
    : Sensor(rocket, refresh_rate, noise_mean, noise_stddev) {
    data_ = {0, 0, 0};
    noise_ = {0, 0, 0};
    bias_ = {0, 0, 0};
}

void Accelerometer::update_data(double tStep) {
    if ((tStep - last_update_tStep_) >= (1 / refresh_rate_)) {
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

Vector3d Accelerometer::get_data() {
    Vector3d ret = data_;
    new_data_ = false;
    return ret;
}

/*****************************************************************************/
/*                       MAGNETOMETER MEMBER FUNCTIONS                       */
/*****************************************************************************/

// TODO implement magnetometer
EmulatedMagnetometerSensor::EmulatedMagnetometerSensor(Rocket& rocket, double refresh_rate, double noise_mean, double noise_stddev)
    : Sensor(rocket, refresh_rate, noise_mean, noise_stddev) { }

void EmulatedMagnetometerSensor::update_data(double tStep) {
    Vector3d north_enu = {0.0, 1.0, 0.0};
    Vector3d north_rf = rocket_.enu2r(north_enu);
    data_ = north_rf;
    new_data_ = true;
}

Vector3d EmulatedMagnetometerSensor::get_data() {
    new_data_ = false;
    return data_;
}

/*****************************************************************************/
/*                        BAROMETER MEMBER FUNCTIONS                         */
/*****************************************************************************/

Barometer::Barometer(Rocket& rocket, double refresh_rate, double noise_mean, double noise_stddev)
    : Sensor(rocket, refresh_rate, noise_mean, noise_stddev) {
    data_ = rocket_.get_r_vect().x();
    bias_ = 0;
    noise_ = 0;
}

void Barometer::update_data(double tStep) {
    if ((tStep - last_update_tStep_) >= (1 / refresh_rate_)) {
        data_ = rocket_.get_r_vect().z();
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

/*****************************************************************************/
/*                      THERMOMOMETER MEMBER FUNCTIONS                       */
/*****************************************************************************/

Thermometer::Thermometer(Rocket& rocket, double refresh_rate, double noise_mean, double noise_stddev)
    : Sensor(rocket, refresh_rate, noise_mean, noise_stddev) { }

void Thermometer::update_data(double tStep) {
    if ((tStep - last_update_tStep_) >= (1 / refresh_rate_)) {
        data_ = Atmosphere::get_temperature(rocket_.get_r_vect().z());
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

double Thermometer::get_data() { return data_; }

/*****************************************************************************/
/*                        GPS SENSOR MEMBER FUNCTIONS                        */
/*****************************************************************************/

EmulatedGPSSensor::EmulatedGPSSensor(Rocket& rocket, double refresh_rate, double noise_mean, double noise_stddev)
    : Sensor(rocket, refresh_rate, noise_mean, noise_stddev) { }

void EmulatedGPSSensor::update_data(double tStep) {
    if ((tStep - last_update_tStep_) >= (1 / refresh_rate_)) {
        data_ = rocket_.get_r_vect();
        new_data_ = true;

        if (inject_noise_) {
            noise_.x() = normal_dist_(generator_);
            noise_.y() = normal_dist_(generator_);
            noise_.z() = normal_dist_(generator_);
            data_ += noise_;
        }

        if (inject_bias_) {
            data_ += bias_;
        }
    }
}

Vector3d EmulatedGPSSensor::get_data() {
    return data_;
}
