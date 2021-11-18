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

    float temperature;
    float geopotential_to_geometric = 86.0 - 84.852;
    altitude = altitude/1000;
    if (altitude < 11.0) {
        temperature = 288.15 - (6.5 * altitude);
    }
    
    else if (altitude < 20.0) {
        temperature = 216.65;
    }
    
    else if (altitude < 32.0) {
        temperature = 196.65 + altitude;
    }

    else if (altitude < 47.0) {
        temperature = 139.05 + (2.8 * altitude);
    }

    else if (altitude < 51.0) {
        temperature = 270.65;
    }

    else if (altitude < 71.0) {
        temperature = 413.45 - (2.8 * altitude);
    }

    else if (altitude < 84.852) {
        temperature = 356.65 - (2.0 * altitude);
    }

    else if (altitude < (91 - geopotential_to_geometric)) {
        temperature = 186.8673;
    }

    else if (altitude < (110 - geopotential_to_geometric)) {
        temperature = 263.1905 - 76.3232 * sqrt(1 - pow(((altitude + geopotential_to_geometric) - 91) / -19.9429, 2.0));
    }

    else if (altitude < (120 - geopotential_to_geometric)) {
        temperature = 240 + 12 * ((altitude + geopotential_to_geometric) - 110);
    }

    else if (altitude < (1000 - geopotential_to_geometric)){
        temperature = 1000 - 640 * exp(-0.01875 * (((altitude + geopotential_to_geometric) - 120) * (6356.766 + 120) / (6356.766 + (altitude + geopotential_to_geometric))));    }
    }
    return temperature;
}

/**
 * @brief Pressure getter function based on altitude
 *
 * @param altitude Altitude above sea level in meters
 */
double Atmosphere::get_pressure(double altitude) {
    float pressure;
    float geopotential_to_geometric = 86.0 - 84.852;
    altitude = altitude/1000;

    if (altitude < 11) {
        pressure = 101325.0 * pow([288.15 / (288.15 - 6.5 * altitude)], (34.1632 / -6.5))
    }

    else if (altitude < 20){
        pressure = 22632.06 * exp(-34.1632 * (altitude - 11) / 216.65);
    }

    else if (altitude < 32){
        pressure = 5474.889 * pow([616.65 / (216.65 + 2.8 * (altitude -20))], 34.1632);
    }

    else if (altitue < 47){
        pressure = 868.0187 * pow([228.65 / (216.65 + 2.8 * (h - 32) )], (34.1632 / 28));
    }

    else if (altitude < 51){
        pressure = 110.9063 * exp(-34.1632 * (h -47) / 270.65);
    }

    else if (altitude < 71){
        pressure = 66.93887 * pow([270.65 / (214.65 -2 * (h -71))], (34.1632 / -2.8));
    }

    else if (altitude < 84.852){
                pressure = 3.956420 * pow([214.65 / (214.65 - 2 * (h -71))], (34.1632 / -2));
    }
//86k to 1000k formula not sure yet
    return altitude;
}

/**
 * @brief Density getter function based on altitude
 *
 * @param altitude Altitude above sea level in meters
 */
double Atmosphere::get_density(double altitude) {
    return altitude;
}