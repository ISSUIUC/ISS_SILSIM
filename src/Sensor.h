/**
 * @file        Sensor.h
 * @authors     Ayberk Yaraneri
 *
 * @brief       Class definition for Sensor classes
 *
 * These Sensor classes encapsulate typical sensors found on Rocket avionics
 * like accelerometers and gyroscopes. This is the mechanism through which
 * the flight software being tested can obtain information of the simulated
 * rocket. The classes provide a modular a means of injecting noise and bias
 * along with other inaccuracies to make sensor measurements behave closer to
 * real hardware rather than simply providing ground truth.
 *
 */

#pragma once

#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <random>
#include <string>
#include <vector>

#include "Rocket.h"
#include "Vector3.h"

class Sensor {
   public:
    Sensor(std::string name, Rocket& rocket, double refresh_rate,
           double noise_mean = 0.0f, double noise_stddev = 0.1f)
        : name_(name),
          rocket_(rocket),
          refresh_rate_(refresh_rate),
          last_update_tStep_(0),
          normal_dist_(noise_mean, noise_stddev){};

    bool is_new_data() { return new_data_; };
    std::string get_name() { return name_; };

    virtual void update_data(double tStep);
    virtual void get_data(Vector3& data);
    virtual double get_data();

    // Noise/bias injection control
    void enable_noise_injection() { inject_noise_ = true; };
    void disable_noise_injection() { inject_noise_ = false; };
    void enable_bias_injection() { inject_bias_ = true; };
    void disable_bias_injection() { inject_bias_ = false; };

   protected:
    std::string name_;

    Rocket& rocket_;

    double refresh_rate_;
    double last_update_tStep_;
    bool new_data_;

    // Normal distribution noise generation
    std::default_random_engine generator_;
    std::normal_distribution<double> normal_dist_;

    // Whether to inject constant bias or gaussian noise to measurement
    bool inject_bias_ = false;
    bool inject_noise_ = false;
};

class Gyroscope : public Sensor {
   public:
    Gyroscope(std::string name, Rocket& rocket, double refresh_rate,
              double noise_mean = 0.0f, double noise_stddev = 0.1f);
    void update_data(double tStep);
    void get_data(Vector3& data);

    void set_constant_bias(Vector3 bias) { bias_ = bias; };

   private:
    Vector3 data_;  // The sensor's current reading

    Vector3 noise_;  // Noise vector to be added to measurement

    Vector3 bias_;  // Constant bias vector to be added to measurement
};

class Accelerometer : public Sensor {
   public:
    Accelerometer(std::string name, Rocket& rocket, double refresh_rate,
                  double noise_mean = 0.0f, double noise_stddev = 0.1f);
    void update_data(double tStep);
    void get_data(Vector3& data);

    void set_constant_bias(Vector3 bias) { bias_ = bias; };

   private:
    Vector3 data_;  // The sensor's current reading

    Vector3 noise_;  // Noise vector to be added to measurement

    Vector3 bias_;  // Constant bias vector to be added to measurement
};

// TODO: Implement functions for both altitude and pressure measurements (and
// specify units)
class Barometer : public Sensor {
   public:
    Barometer(std::string name, Rocket& rocket, double refresh_rate,
              double noise_mean = 0.0f, double noise_stddev = 0.1f);
    void update_data(double tStep);
    double get_data();

    void set_constant_bias(double bias) { bias_ = bias; };

    double get_altitude();

   private:
    double data_;  // The sensor's current reading

    double noise_;  // Noise value to be added to measurement

    double bias_;  // Constant bias value to be added to measurement

};

#endif
