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

#include <Eigen/Dense>

using Eigen::Vector3d;

class Atmosphere {

   public:
    /********************** Get Atmospheric Values ****************************/
    static double get_temperature(double altitude);
    static double get_pressure(double altitude);
    static double get_density(double altitude);
    static double get_speed_of_sound(double altitude);
    static double get_geometric_to_geopotential(double altitude);

    Vector3d get_wind_vector();

    /************************** Get Parameters ********************************/
    Vector3d get_wind_direction() {return wind_direction_;};
    double get_wind_magnitude() {return wind_magnitude_;};

    /************************** Set Parameters ********************************/
    void set_wind_direction(Vector3d vec) {wind_direction_ = vec;};
    void set_wind_magnitude(double mag) {wind_magnitude_ = mag;};

   private:

    Vector3d wind_direction_{-1.0, 0.0, 0.0};
    double wind_magnitude_{0.0};
};

#endif
