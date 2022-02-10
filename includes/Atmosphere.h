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

class Atmosphere {
   public:
        static double get_temperature(double altitude);
        static double get_pressure(double altitude);
        static double get_density(double altitude);
        static double get_geometric_to_geopotential(double altitude);
        static double get_wind_direction(double altitude);
        static double get_wind_speed(double altitude);
        static void create_table(std::string filename);
    private:
        std::vector<std::vector<double>> wind_data_table;
};

#endif