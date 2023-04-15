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

#include <Eigen/Dense>
#include <random>
#include <string>
#include <vector>

#include "Rocket/Rocket.h"

using Eigen::Vector3d;

/*****************************************************************************/
/*                             SENSOR BASE CLASS                             */
/*****************************************************************************/

class Sensor {
   public:
    Sensor(Rocket& rocket, double refresh_rate, double noise_mean = 0.0f, double noise_stddev = 0.1f)
        : rocket_(rocket),
          refresh_rate_(refresh_rate),
          last_update_tStep_(0),
          normal_dist_(noise_mean, noise_stddev){};

    bool is_new_data() const { return new_data_; };

    virtual void update_data(double tStep) = 0;

    // Noise/bias injection control
    void enable_noise_injection() { inject_noise_ = true; };
    void disable_noise_injection() { inject_noise_ = false; };
    void enable_bias_injection() { inject_bias_ = true; };
    void disable_bias_injection() { inject_bias_ = false; };

   protected:
    Rocket& rocket_;

    double refresh_rate_;
    double last_update_tStep_;
    bool new_data_ = false;

    // Normal distribution noise generation
    std::default_random_engine generator_;
    std::normal_distribution<double> normal_dist_;

    // Whether to inject constant bias or gaussian noise to measurement
    bool inject_bias_ = false;
    bool inject_noise_ = false;
};

/*****************************************************************************/
/*                        GYROSCOPE CLASS DEFINITION                         */
/*****************************************************************************/

class GyroscopeSensor : public Sensor {
   public:
    GyroscopeSensor(Rocket& rocket, double refresh_rate, double noise_mean = 0.0f, double noise_stddev = 0.1f);
    void update_data(double tStep) override;
    Vector3d get_data();

    void set_constant_bias(Vector3d bias) { bias_ = bias; };

   private:
    Vector3d data_;  // The sensor's current reading
    Vector3d noise_;  // Noise vector to be added to measurement
    Vector3d bias_;  // Constant bias vector to be added to measurement
};

/*****************************************************************************/
/*                      ACCELEROMETER CLASS DEFINITION                       */
/*****************************************************************************/

class Accelerometer : public Sensor {
   public:
    Accelerometer(Rocket& rocket, double refresh_rate, double noise_mean = 0.0f, double noise_stddev = 0.1f);
    void update_data(double tStep) override;
    Vector3d get_data();

    void set_constant_bias(Vector3d bias) { bias_ = bias; };

   private:
    Vector3d data_;  // The sensor's current reading
    Vector3d noise_;  // Noise vector to be added to measurement
    Vector3d bias_;  // Constant bias vector to be added to measurement
};

/*****************************************************************************/
/*                       MAGNETOMETER CLASS DEFINITION                       */
/*****************************************************************************/

class EmulatedMagnetometerSensor : public Sensor {
   public:
    EmulatedMagnetometerSensor(Rocket& rocket, double refresh_rate, double noise_mean = 0.0f, double noise_stddev = 0.1f);
    void update_data(double tStep) override;
    Vector3d get_data();

    void set_constant_bias(Vector3d bias) { bias_ = bias; };

   private:
    Vector3d data_;  // The sensor's current reading
    Vector3d noise_;  // Noise vector to be added to measurement
    Vector3d bias_;  // Constant bias vector to be added to measurement
};

/*****************************************************************************/
/*                         BAROMETER CLASS DEFINITION                        */
/*****************************************************************************/

// TODO: Implement functions for both altitude and pressure measurements (and
// specify units)
class Barometer : public Sensor {
   public:
    Barometer(Rocket& rocket, double refresh_rate, double noise_mean = 0.0f, double noise_stddev = 0.1f);
    void update_data(double tStep) override;
    double get_data();

    void set_constant_bias(double bias) { bias_ = bias; };

   private:
    double data_;  // The sensor's current reading
    double noise_;  // Noise value to be added to measurement
    double bias_;  // Constant bias value to be added to measurement
};

/*****************************************************************************/
/*                       THERMOMETER CLASS DEFINITION                        */
/*****************************************************************************/

class Thermometer : public Sensor {
   public:
    Thermometer(Rocket& rocket, double refresh_rate, double noise_mean = 0.0f, double noise_stddev = 0.1f);
    void update_data(double tStep) override;
    double get_data();

   private:
    double data_;  // The sensor's current reading
    double noise_;  // Noise value to be added to measurement
    double bias_;  // Constant bias value to be added to measurement
};

/*****************************************************************************/
/*                        GPS SENSOR CLASS DEFINITION                        */
/*****************************************************************************/

class EmulatedGPSSensor : public Sensor {
   public:
    EmulatedGPSSensor(Rocket& rocket, double refresh_rate, double noise_mean = 0.0f, double noise_stddev = 0.1f);
    void update_data(double tStep) override;
    Vector3d get_data();

    void set_constant_bias(Vector3d bias) { bias_ = bias; };

   private:
    Vector3d data_;  // The sensor's current reading
    Vector3d noise_;  // Noise vector to be added to measurement
    Vector3d bias_;  // Constant bias vector to be added to measurement
};

/*****************************************************************************/
/*                             UTILITY FUNCTIONS                             */
/*****************************************************************************/

Vector3d randomize_vector(std::default_random_engine& generator,
                          std::normal_distribution<double>& dist);

#endif
