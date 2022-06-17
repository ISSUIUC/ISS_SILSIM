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

#include "Atmosphere.h"

using Eigen::Vector3d;

void Sensor::update_data(double tStep) {
    (void)tStep;
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

Gyroscope::Gyroscope(std::string name, Rocket& rocket, double refresh_rate,
                     spdlog_basic_sink_ptr silsim_sink, double noise_mean,
                     double noise_stddev)
    : Sensor(name, rocket, refresh_rate, noise_mean, noise_stddev) {
    data_ = {0, 0, 0};
    noise_ = {0, 0, 0};
    bias_ = {0, 0, 0};

    if (silsim_sink) {
        sensor_logger_ =
            std::make_shared<spdlog::logger>("Gyroscope:" + name, silsim_sink);
        sensor_logger_->info("DATALOG_FORMAT," + datalog_format_string);
    }
}

void Gyroscope::update_data(double tStep) {
    if ((tStep - last_update_tStep_) >= (1 / refresh_rate_)) {
        data_ = rocket_.enu2r(rocket_.get_w_vect());
        last_update_tStep_ = tStep;
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

void Gyroscope::get_data(Vector3d& data) {
    data = data_;
    new_data_ = false;
}

void Gyroscope::log_sensor_state(double tStamp) {
    if (sensor_logger_) {
        // clang-format off
        std::stringstream datalog_ss;

        datalog_ss << "DATA,"
                   << tStamp << ","
                   << data_.x() << ","
                   << data_.y() << ","
                   << data_.z();

        sensor_logger_->info(datalog_ss.str());
        // clang-format on
    }
}

/*****************************************************************************/
/*                      ACCELEROMETER MEMBER FUNCTIONS                       */
/*****************************************************************************/

Accelerometer::Accelerometer(std::string name, Rocket& rocket,
                             double refresh_rate,
                             spdlog_basic_sink_ptr silsim_sink,
                             double noise_mean, double noise_stddev)
    : Sensor(name, rocket, refresh_rate, noise_mean, noise_stddev) {
    data_ = {0, 0, 0};
    noise_ = {0, 0, 0};
    bias_ = {0, 0, 0};

    if (silsim_sink) {
        sensor_logger_ = std::make_shared<spdlog::logger>(
            "Accelerometer:" + name, silsim_sink);
        sensor_logger_->info("DATALOG_FORMAT," + datalog_format_string);
    }
}

void Accelerometer::update_data(double tStep) {
    if ((tStep - last_update_tStep_) >= (1 / refresh_rate_)) {
        // Subtract the gravity vector from the rocket's total acceleration to
        // yield specific force
        Vector3d total_accel = rocket_.enu2r(rocket_.get_r_ddot());
        Vector3d gravity_rocket_frame = rocket_.gravity_vector_rf() * 9.81;
        Vector3d specific_force = total_accel - gravity_rocket_frame;

        data_ = specific_force/9.81;
        last_update_tStep_ = tStep;
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

void Accelerometer::log_sensor_state(double tStamp) {
    if (sensor_logger_) {
        // clang-format off
        std::stringstream datalog_ss;

        datalog_ss << "DATA,"
                   << tStamp << ","
                   << data_.x() << ","
                   << data_.y() << ","
                   << data_.z();

        sensor_logger_->info(datalog_ss.str());
        // clang-format on
    }
}

/*****************************************************************************/
/*                       MAGNETOMETER MEMBER FUNCTIONS                       */
/*****************************************************************************/

// TODO implement magnetometer
Magnetometer::Magnetometer(std::string name, Rocket& rocket,
                           double refresh_rate,
                           spdlog_basic_sink_ptr silsim_sink, double noise_mean,
                           double noise_stddev)
    : Sensor(name, rocket, refresh_rate, noise_mean, noise_stddev) {
    if (silsim_sink) {
        sensor_logger_ = std::make_shared<spdlog::logger>(
            "Magnetometer:" + name, silsim_sink);
        sensor_logger_->info("DATALOG_FORMAT," + datalog_format_string);
    }
}

void Magnetometer::update_data(double tStep) {
    Vector3d north_enu = {0.0, 1.0, 0.0};
    Vector3d north_rf = rocket_.enu2r(north_enu);
    data_ = north_rf;
    last_update_tStep_ = tStep;
    new_data_ = true;
}

void Magnetometer::get_data(Vector3d& data) {
    data = data_;
    new_data_ = false;
}

void Magnetometer::log_sensor_state(double tStamp) {
    if (sensor_logger_) {
        // clang-format off
        std::stringstream datalog_ss;

        datalog_ss << "DATA,"
                   << tStamp << ","
                   << data_.x() << ","
                   << data_.y() << ","
                   << data_.z();

        sensor_logger_->info(datalog_ss.str());
        // clang-format on
    }
}

/*****************************************************************************/
/*                        BAROMETER MEMBER FUNCTIONS                         */
/*****************************************************************************/

Barometer::Barometer(std::string name, Rocket& rocket, double refresh_rate,
                     spdlog_basic_sink_ptr silsim_sink, double noise_mean,
                     double noise_stddev)
    : Sensor(name, rocket, refresh_rate, noise_mean, noise_stddev) {
    // Barometer data adjusted to be MSL (New Mexico MSL altitude is 1401)
    alt = rocket_.get_r_vect().z() + 1401;
    bias_ = 0;
    noise_ = 0;

    data_ = Atmosphere::get_pressure(alt);

    if (silsim_sink) {
        sensor_logger_ =
            std::make_shared<spdlog::logger>("Barometer:" + name, silsim_sink);
        sensor_logger_->info("DATALOG_FORMAT," + datalog_format_string);
    }
}

void Barometer::update_data(double tStep) {
    if ((tStep - last_update_tStep_) >= (1 / refresh_rate_)) {
        // Barometer data adjusted to be MSL (New Mexico MSL altitude is 1401)
        alt = rocket_.get_r_vect().z() + 1401;
        last_update_tStep_ = tStep;
        new_data_ = true;

        if (inject_noise_) {
            noise_ = normal_dist_(generator_);
            alt += noise_;
        }

        if (inject_bias_) {
            alt += bias_;
        }
        data_ = Atmosphere::get_pressure(alt);
    }
}

double Barometer::get_data() {
    new_data_ = false;
    return data_;
}

void Barometer::log_sensor_state(double tStamp) {
    if (sensor_logger_) {
        // clang-format off
        std::stringstream datalog_ss;

        datalog_ss << "DATA,"
                   << tStamp << ","
                   << data_;

        sensor_logger_->info(datalog_ss.str());
        // clang-format on
    }
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

Thermometer::Thermometer(std::string name, Rocket& rocket, double refresh_rate,
                         spdlog_basic_sink_ptr silsim_sink, double noise_mean,
                         double noise_stddev)
    : Sensor(name, rocket, refresh_rate, noise_mean, noise_stddev) {
    if (silsim_sink) {
        sensor_logger_ = std::make_shared<spdlog::logger>("Thermometer:" + name,
                                                          silsim_sink);
        sensor_logger_->info("DATALOG_FORMAT," + datalog_format_string);
    }
}

void Thermometer::update_data(double tStep) {
    if ((tStep - last_update_tStep_) >= (1 / refresh_rate_)) {
        data_ = Atmosphere::get_temperature(rocket_.get_r_vect().z());
        last_update_tStep_ = tStep;
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

void Thermometer::log_sensor_state(double tStamp) {
    if (sensor_logger_) {
        // clang-format off
        std::stringstream datalog_ss;

        datalog_ss << "DATA,"
                   << tStamp << ","
                   << data_;

        sensor_logger_->info(datalog_ss.str());
        // clang-format on
    }
}

/*****************************************************************************/
/*                        GPS SENSOR MEMBER FUNCTIONS                        */
/*****************************************************************************/

GPSSensor::GPSSensor(std::string name, Rocket& rocket, double refresh_rate,
                     spdlog_basic_sink_ptr silsim_sink, double noise_mean,
                     double noise_stddev)
    : Sensor(name, rocket, refresh_rate, noise_mean, noise_stddev) {
    if (silsim_sink) {
        sensor_logger_ =
            std::make_shared<spdlog::logger>("GPSSensor:" + name, silsim_sink);
        sensor_logger_->info("DATALOG_FORMAT," + datalog_format_string);
    }
}

void GPSSensor::update_data(double tStep) {
    if ((tStep - last_update_tStep_) >= (1 / refresh_rate_)) {
        data_ = rocket_.get_r_vect();
        last_update_tStep_ = tStep;
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

void GPSSensor::get_data(Vector3d& data) { data = data_; }

void GPSSensor::log_sensor_state(double tStamp) {
    if (sensor_logger_) {
        // clang-format off
        std::stringstream datalog_ss;

        datalog_ss << "DATA,"
                   << tStamp << ","
                   << data_.x() << ","
                   << data_.y() << ","
                   << data_.z();

        sensor_logger_->info(datalog_ss.str());
        // clang-format on
    }
}
