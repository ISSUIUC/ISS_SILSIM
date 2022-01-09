/**
 * @file        Atmosphere.cpp
 * @authors     Kenneth Tochihara
 *
 * @brief       Member function implementations of atmosphere component
 * classes
 *
 * The classes defined here represent atmosphere components. These
 * objects will represent the atmosphere model used for simulation.
 *
 */

#include "Atmosphere.h"

#include <iostream>
#include <string>
#include <vector>
#include <math.h>


/**
 * @brief Temperature getter function based on altitude
 *
 * @param altitude Altitude above sea level in meters
 */
double Atmosphere::get_temperature(double altitude) {

    double temperature;
    altitude_h = get_geometric_to_geopotential(altitude)/1000; //geopotential//
    altitude_z = altitude/1000; //geometric//
    if (altitude_h < 11.0) {
        temperature = 288.15 - (6.5 * altitude_h);
    }
    
    else if (altitude_h < 20.0) {
        temperature = 216.65;
    }
    
    else if (altitude_h < 32.0) {
        temperature = 196.65 + altitude_h;
    }

    else if (altitude_h < 47.0) {
        temperature = 139.05 + (2.8 * altitude_h);
    }

    else if (altitude_h < 51.0) {
        temperature = 270.65;
    }

    else if (altitude_h < 71.0) {
        temperature = 413.45 - (2.8 * altitude_h);
    }

    else if (altitude_h < 84.852) {
        temperature = 356.65 - (2.0 * altitude_h);
    }

    else if (aaltitude_z < 91) {
        temperature = 186.8673;
    }

    else if (altitude_z < 110) {
        temperature = 263.1905 - 76.3232 * sqrt(1 - pow((altitude_z - 91) / -19.9429, 2.0));
    }

    else if (altitude_z < 120) {
        temperature = 240 + 12 * (altitude_z - 110);
    }

    else if (altitude_z < 1000) {
        temperature = 1000 - 640 * exp(-0.01875 * ((altitude_z - 120) * (6356.766 + 120) / (6356.766 + altitude_z)));
    }
    else {
        throw std::runtime_error("exceding caculatable altitude");
    }
    return temperature;
}

/**
 * @brief Pressure getter function based on altitude
 *
 * @param altitude Altitude above sea level in meters
 */
double Atmosphere::get_pressure(double altitude) {
    double pressure;
    altitude = get_geometric_to_geopotential(altitude)/1000;

    if (altitude < 11) {
        pressure = 101325.0 * pow((288.15 / (288.15 - 6.5 * altitude)), (34.1632 / -6.5))
    }

    else if (altitude < 20){
        pressure = 22632.06 * exp(-34.1632 * (altitude - 11) / 216.65);
    }

    else if (altitude < 32){
        pressure = 5474.889 * pow((216.65 / (216.65 + 2.8 * (altitude -20))), 34.1632);
    }

    else if (altitue < 47){
        pressure = 868.0187 * pow((228.65 / (228.65 + 2.8 * (altitude - 32))), (34.1632 / 28));
    }

    else if (altitude < 51){
        pressure = 110.9063 * exp(-34.1632 * (altitude -47) / 270.65);
    }

    else if (altitude < 71){
        pressure = 66.93887 * pow((270.65 / (270.65 - 2.8 * (altitude - 51))), (34.1632 / -2.8));
    }

    else if (altitude < 84.852){
                pressure = 3.956420 * pow((214.65 / (214.65 - 2 * (altitude -71))), (34.1632 / -2));
    }
    else {
        throw std::runtime_error("exceding caculatable altitude");
    }
//86k to 1000k formula not sure yet
    return pressure;
}

/**
 * @brief Density getter function based on altitude
 *
 * @param altitude Altitude above sea level in meters
 */
double Atmosphere::get_density(double altitude) {
    double R = 287.053;
    double pressure = get_preessure(altitude);
    double temperature = get_temperature(altitude);
    altitude = altitude/1000;

    if (altitude < 84.853){
        density = P/(R*temperature);
    }

    return density;
}


double Atmosphere::get_geometric_to_geopotential(double altitude) {
    double r = 6371000; // r means the radius of Earth//
    double geopotential = (r * altitude) /(r + altitude); 

    return geopotential;
}