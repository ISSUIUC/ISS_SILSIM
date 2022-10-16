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
#include <fstream>

#ifdef HILSIM
#include <windows.h>
#endif

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
/*                        SERIAL MEMBER FUNCTIONS                            */
/*****************************************************************************/

void SerialComm::serial_open() { 

    #ifdef linux
    this->serial_file_.open(this->port_);
    #endif

    #ifdef _WIN32
    this->serial_file_ = CreateFile(this->port, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    // DCB serialParams = { 0 };
    // serialParams.DCBlength = sizeof(serialParams);

    // GetCommState(serialHandle, &serialParams);
    // serialParams.BaudRate = CBR_9600;
    // serialParams.ByteSize = 8;
    // serialParams.StopBits = ONESTOPBIT;
    // serialParams.Parity = NOPARITY;
    // SetCommState(serialHandle, &serialParams);

    // Set timeouts
    // COMMTIMEOUTS timeout = { 0 };
    // timeout.ReadIntervalTimeout = 50;
    // timeout.ReadTotalTimeoutConstant = 50;
    // timeout.ReadTotalTimeoutMultiplier = 50;
    // timeout.WriteTotalTimeoutConstant = 50;
    // timeout.WriteTotalTimeoutMultiplier = 10;

    // SetCommTimeouts(serialHandle, &timeout);
    #endif
}

void SerialComm::serial_add_data(char* data) {
    std::cout << "BUFFER LENGTH: " << strlen(this->buffer_);

    memcpy(this->buffer_ + strlen(this->buffer_), data, strlen(data));
}

void SerialComm::serial_write() { 
    // Send data
    #ifdef linux
    this->serial_file_.write((this->buffer_), sizeof(this->buffer_));
    #endif

    #ifdef _WIN32
    WriteFile(this->serial_file_, this->buffer_,sizeof(buffer_))
    #endif

    //clear buffer
    memset(this->buffer_, 0, sizeof(this->buffer_));

    return;
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

        // Send this data to serial if HILSIM is defined
        #ifdef HILSIM
        std::string serial_port = "/dev/ttyACM0";
        SerialComm comm = SerialComm(serial_port);
        char data[1024];
        sprintf(data, "%f,%f,%f", data_.x(), data_.y(), data_.z());
        comm.serial_open();
        comm.serial_add_data(data);
        comm.serial_write();

        // std::ofstream serial_file;
        // serial_file.open(serial_port);
        // serial_file.write("HI", 2);

        // std::ofstream serial_file = serial_open(serial_port);
        // std::cout << "Gyroscope: " << "," << data_.x() << "," << data_.y() << "," << data_.z(); 
        #endif
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
    data_ = rocket_.get_r_vect().x();
    bias_ = 0;
    noise_ = 0;

    if (silsim_sink) {
        sensor_logger_ =
            std::make_shared<spdlog::logger>("Barometer:" + name, silsim_sink);
        sensor_logger_->info("DATALOG_FORMAT," + datalog_format_string);
    }
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
