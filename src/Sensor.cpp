/**
 * @file 		Sensor.cpp
 * @authors 	Ayberk Yaraneri
 *
 * @brief 		Member function implementations for Sensor classes
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
#include <vector>

#include "Rocket.h"

Gyroscope::Gyroscope(std::string name, Rocket& rocket, double refresh_rate,
                     double noise_mean, double noise_stddev)
    : Sensor(name, rocket, refresh_rate, noise_mean, noise_stddev) {
    data_ = Vector3();
}

void Gyroscope::update_data(double tStep) {
    if ((tStep - last_update_tStep_) >= (1 / refresh_rate_)) {
        rocket_.get_w_vect(data_);
        rocket_.i2r(data_);
        new_data_ = true;

        if (inject_noise_) {
            noise_.randomize(generator_, normal_dist_);
            data_ += noise_;
        }

        if (inject_bias_) {
            data_ += bias_;
        }
    }
}

void Gyroscope::get_data(Vector3& data) {
    data = data_;
    new_data_ = false;
}

Accelerometer::Accelerometer(std::string name, Rocket& rocket,
                             double refresh_rate, double noise_mean,
                             double noise_stddev)
    : Sensor(name, rocket, refresh_rate, noise_mean, noise_stddev) {
    data_ = Vector3();
}

void Accelerometer::update_data(double tStep) {
    if ((tStep - last_update_tStep_) >= (1 / refresh_rate_)) {
        rocket_.get_r_ddot(data_);
        rocket_.i2r(data_);
        new_data_ = true;

        if (inject_noise_) {
            noise_.randomize(generator_, normal_dist_);
            data_ += noise_;
        }

        if (inject_bias_) {
            data_ += bias_;
        }
    }
}

void Accelerometer::get_data(Vector3& data) {
    data = data_;
    new_data_ = false;
}

Barometer::Barometer(std::string name, Rocket& rocket, double refresh_rate,
                     double noise_mean, double noise_stddev)
    : Sensor(name, rocket, refresh_rate, noise_mean, noise_stddev) {
    data_ = rocket_.get_r_vect().x;
}

void Barometer::update_data(double tStep) {
    if ((tStep - last_update_tStep_) >= (1 / refresh_rate_)) {
        data_ = rocket_.get_r_vect().x;
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

void Barometer::get_data(double& data) {
    data = data_;
    new_data_ = false;
}
