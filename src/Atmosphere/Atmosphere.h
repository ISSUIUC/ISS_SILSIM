/**
 * @file        Atmosphere.h
 * @authors     Kenneth Tochihara
 *              Buffet Lee
 *              Ayberk Yaraneri
 *
 * @brief       Class definitions for the atmosphere model
 *
 * The classes defined here represent atmosphere components. These
 * objects will represent the atmosphere model used for simulation.
 *
 */

#pragma once

#ifndef _ATMOSPHERE_H_
#define _ATMOSPHERE_H_

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <random>

// Shortening the typename for   a e s t h e t i c s
typedef std::shared_ptr<spdlog::sinks::basic_file_sink_mt>
    spdlog_basic_sink_ptr;

using Eigen::Vector3d;

class Atmosphere {
   public:
    Atmosphere(spdlog_basic_sink_ptr silsim_sink,
               double wind_direction_variance_mean = 0.0f,
               double wind_direction_variance_stddev = 0.1f,
               double wind_magnitude_variance_mean = 0.0f,
               double wind_magnitude_variance_stddev = 0.1f)
        : direction_normal_dist_(wind_direction_variance_mean,
                                 wind_direction_variance_stddev),
          magnitude_normal_dist_(wind_magnitude_variance_mean,
                                 wind_magnitude_variance_stddev) {
              if (silsim_sink) {
                  atmosphere_logger_ = std::make_shared<spdlog::logger>("Atmosphere", silsim_sink);
                  atmosphere_logger_->info("DATALOG_FORMAT," + datalog_format_string);
              }
          };

    // -------------------------- Get Parameters ---------------------------- //
    Vector3d get_nominal_wind_direction() { return nominal_wind_direction_; };
    double get_nominal_wind_magnitude() { return nominal_wind_magnitude_; };

    // -------------------------- Set Parameters ---------------------------- //
    void set_nominal_wind_direction(Vector3d vec) {
        nominal_wind_direction_ = vec.normalized();
    };
    void set_nominal_wind_magnitude(double mag) {
        nominal_wind_magnitude_ = mag;
    };

    // ---------------------- Get Atmospheric Values ------------------------ //
    static double get_temperature(double altitude);
    static double get_pressure(double altitude);
    static double get_density(double altitude);
    static double get_speed_of_sound(double altitude);
    static double get_geometric_to_geopotential(double altitude);

    // -------------------------- Wind Modeling ----------------------------- //
    Vector3d get_wind_vector(double tStamp);
    void toggle_wind_direction_variance(bool toggle) {
        enable_direction_variance_ = toggle;
    };
    void toggle_wind_magnitude_variance(bool toggle) {
        enable_magnitude_variance_ = toggle;
    };

    void log_atmosphere_state(double tStamp);

   private:
    // Nominal wind without any variance
    Vector3d nominal_wind_direction_{-1.0, 0.0, 0.0};
    double nominal_wind_magnitude_{0.0};

    // Current wind with variance
    Vector3d current_wind_direction_{-1.0, 0.0, 0.0};
    double current_wind_magnitude_{0.0};

    // Smoothed wind variance applied onto nominal wind
    Vector3d direction_variance_vect_{0.0, 0.0, 0.0};
    double magnitude_variance_val_{0.0};

    // ----------------------- Wind Variance Generation --------------------- //
    bool enable_direction_variance_;
    bool enable_magnitude_variance_;
    std::default_random_engine generator_;
    std::normal_distribution<double> direction_normal_dist_;
    std::normal_distribution<double> magnitude_normal_dist_;

    // Variance update management. Prevents wind from changing too often
    double last_variance_update_{0.0};
    const double variance_update_rate_{3.0};

    // The generated random wind variance before smoothing
    Vector3d generated_direction_variance_{0.0, 0.0, 0.0};
    double generated_magnitude_variance_{0.0};

    // ----------------------------- Data Logging  -------------------------- //
    std::shared_ptr<spdlog::logger> atmosphere_logger_;
    const std::string datalog_format_string =
        "timestamp,current_wind_x,current_wind_y,current_wind_z,current_wind_magnitude";
};

#endif
