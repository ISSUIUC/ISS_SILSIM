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
#ifdef _WIN32
#include <windows.h>
#endif
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

    #ifdef linux
    this->serial_file_.open(this->port_);
    #endif
    #endif

    #ifdef _WIN32
    this->serial_file_ = CreateFileA(com_path, 
                                    GENERIC_READ | GENERIC_WRITE,
                                    FILE_SHARE_READ | FILE_SHARE_WRITE,
                                    0, // No security
                                    OPEN_EXISTING,
                                    0, // No threads
                                    NULL); //
    if (this->serial_file_ == INVALID_HANDLE_VALUE) {
        printf("Error Opening Serial Port");
        printf("Error: %d", GetLastError());
    }
    #endif

    // Clear Buffer Memory
    memset(this->buffer_, 0, sizeof(this->buffer_));

}

void SerialComm::serial_add_data(char* data) {
    memcpy(this->buffer_ + strlen(this->buffer_), data, strlen(data));
    // printf(this->buffer_);

}

void SerialComm::serial_write() {
    #ifdef linux 
    // Send data and clear buffer
    this->serial_file_.write((this->buffer_), sizeof(this->buffer_));
    #endif

    #ifdef _WIN32
    DWORD bytes_written;
    WriteFile(this->serial_file_, this->buffer_,sizeof(buffer_),&bytes_written,NULL);
    #endif

    memset(this->buffer_, 0, sizeof(this->buffer_));
    return;
}

void SerialComm::serial_read() {

    std::filebuf* inbuf = this->serial_file_.rdbuf();
    char c = inbuf->sbumpc();
    while (c != EOF) {
        std::cout << c;
    }
}

void SerialComm::serial_close() {
    #ifdef linux
    this->serial_file_.close();
    #endif

    #ifdef _WIN32
    CloseHandle(this->serial_file_);
    #endif
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

void Gyroscope::log_sensor_state(double tStamp, char* data_buff) {
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
        if (data_buff) {
            sprintf(data_buff + strlen(data_buff), "Gyroscope:%s %f,%f,%f\n", this->name_, data_.x(), data_.y(), data_.z());
        }
        // #ifdef HILSIM
        // std::string serial_port = "/dev/ttyACM0";
        // SerialComm comm = SerialComm(serial_port);
        // char data[1024];
        // sprintf(data, "Gyroscope: %f,%f,%f\n", data_.x(), data_.y(), data_.z());
        // comm.serial_open();
        // comm.serial_add_data(data);
        // comm.serial_write();
        // comm.serial_close();
        // comm.serial_read();
        // comm.serial_close();
        // #endif
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

void Accelerometer::log_sensor_state(double tStamp, char* data_buff) {
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

        if (data_buff) {
            sprintf(data_buff + strlen(data_buff), "Accelerometer: %f,%f,%f\n", data_.x(), data_.y(), data_.z());
        }
        
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

void Magnetometer::log_sensor_state(double tStamp, char* data_buff) {
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

        if (data_buff) {
            sprintf(data_buff + strlen(data_buff), "Magnetometer: %f,%f,%f\n", data_.x(), data_.y(), data_.z());
        }
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

void Barometer::log_sensor_state(double tStamp, char* data_buff) {
    if (sensor_logger_) {
        // clang-format off
        std::stringstream datalog_ss;

        datalog_ss << "DATA,"
                   << tStamp << ","
                   << data_;

        sensor_logger_->info(datalog_ss.str());
        // clang-format on

        if (data_buff) {
            sprintf(data_buff + strlen(data_buff), "Barometer: %f\n", data_);
        }
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

void Thermometer::log_sensor_state(double tStamp, char* data_buff) {
    if (sensor_logger_) {
        // clang-format off
        std::stringstream datalog_ss;

        datalog_ss << "DATA,"
                   << tStamp << ","
                   << data_;

        sensor_logger_->info(datalog_ss.str());
        // clang-format on

        if (data_buff) {
            sprintf(data_buff + strlen(data_buff), "Thermometer: %f\n", data_);
        }
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

void GPSSensor::log_sensor_state(double tStamp, char* data_buff) {
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

        if (data_buff) {
            sprintf(data_buff + strlen(data_buff), "GPS: %f,%f,%f\n", data_.x(), data_.y(), data_.z());
        }
    }
}
