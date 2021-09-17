/**
 * @file        Atmosphere.h
 * @authors     Kenneth Tochihara
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

#include <string>
#include <vector>

#include "Vector3.h"

class Atmosphere {
   public:
    Atmosphere(double temperature_altitude, double pressure_altitude, double launch_altitude)
        : temperature_altitude_(temperature_altitude),
          pressure_altitude_(pressure_altitude),
          launch_altitude(launch_altitude){};

    double get_temperature(double altitude);
    double get_pressure(double altitude);
    double get_density(double altitude);

   private:
    double temperature_altitude_;
    double pressure_altitude_;
    double launch_altitude;
};

#endif