/**
 * @file 		Propulsion.cpp
 * @authors 	Ayberk Yaraneri
 *
 * @brief 		Member function implementations of propulsion component
 * classes
 *
 * The classes defined here represent rocket propulsion components. While these
 * objects will usually represent rocket motors, they can also be defined to
 * represent components like gas thrusters or other force enducing mechanisms.
 *
 */

#include "Propulsion.h"

#include <iostream>
#include <string>
#include <vector>

/**
 * @brief Transitions state of SolidMotor to ignited (producing thrust)
 *
 * @param tStamp Current simulation timestamp
 */

void SolidMotor::ignite(double tStamp) {
    ignition_ = true;
    ignition_tStamp_ = tStamp;
    current_thrust_ = thrust_value_;
}

/**
 * @brief Thrust vector getter function (by reference)
 *
 * @param tStamp Current simulation timestamp
 * @param vector Vector reference to overwrite with motor's thrust vector
 */
void SolidMotor::get_thrust(double tStamp, Vector3& vector) {
    if (ignition_ == true) {
        if ((tStamp - ignition_tStamp_) <= max_burn_duration_) {
            vector.x = 0.0;
            vector.y = 0.0;
            vector.z = current_thrust_;
            return;
        }
    }
    vector.x = 0.0;
    vector.y = 0.0;
    vector.z = 0.0;
}

/**
 * @brief Thrust vector getter function (by value)
 *
 * @param tStamp Current simulation timestamp
 * @return Vector3 The motor's current thrust vector
 */
Vector3 SolidMotor::get_thrust(double tStamp) {
    Vector3 vector;
    if (ignition_ == true) {
        if ((tStamp - ignition_tStamp_) <= max_burn_duration_) {
            vector.x = 0.0;
            vector.y = 0.0;
            vector.z = current_thrust_;
            return vector;
        }
    }
    vector.x = 0.0;
    vector.y = 0.0;
    vector.z = 0.0;

    return vector;
}
