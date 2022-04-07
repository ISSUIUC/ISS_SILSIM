/**
 * @file        Propulsion.cpp
 * @authors     Ayberk Yaraneri
 *
 * @brief       Member function implementations of propulsion component
 * classes
 *
 * The classes defined here represent rocket propulsion components. While these
 * objects will usually represent rocket motors, they can also be defined to
 * represent components like gas thrusters or other force enducing mechanisms.
 *
 */

#include "Propulsion.h"

#include <Eigen/Dense>

using Eigen::Vector3d;

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
void SolidMotor::get_thrust(double tStamp, Vector3d& vector) const {
    if (ignition_) {
        if ((tStamp - ignition_tStamp_) <= max_burn_duration_) {
            vector.x() = 0.0;
            vector.y() = 0.0;
            vector.z() = current_thrust_;
            return;
        }
    }
    vector.x() = 0.0;
    vector.y() = 0.0;
    vector.z() = 0.0;
}

/**
 * @brief Thrust vector getter function (by value)
 *
 * @param tStamp Current simulation timestamp
 * @return Vector3d The motor's current thrust vector
 */
Vector3d SolidMotor::get_thrust(double tStamp) const {
    Vector3d vector;
    if (ignition_) {
        if ((tStamp - ignition_tStamp_) <= max_burn_duration_) {
            vector.x() = 0.0;
            vector.y() = 0.0;
            vector.z() = current_thrust_;
            return vector;
        }
    }
    vector.x() = 0.0;
    vector.y() = 0.0;
    vector.z() = 0.0;

    return vector;
}

/**
<<<<<<< HEAD
 * @brief Check if motor is currently burning and producing thrust
 *
 * @param tStamp Current simulation timestamp
 * @return bool True if motor is burning and producing thrust, false otherwise
 */
bool SolidMotor::is_burning(double tStamp) const {
    double burn_end_tStamp = ignition_tStamp_ + max_burn_duration_;

    return (tStamp > burn_end_tStamp);
=======
 * @brief Calculate the mass of propellant currently in motor
 *
 * Performs a simple linear interpolation between the initial motor propellant
 * mass and zero depending on how long motor has been burning
 *
 * @param tStamp Current simulation timestamp
 * @return double Current propellant mass within the motor
 */
double SolidMotor::get_propellant_mass(double tStamp) const {
    if (!ignition_) {
        return initial_propellant_mass_;
    }

    if ((tStamp - ignition_tStamp_) > max_burn_duration_) {
        return 0.0;
    }

    return initial_propellant_mass_ *
           (1.0 - ((tStamp - ignition_tStamp_) / max_burn_duration_));
>>>>>>> AV-547-solid-propellant-weight-change
}
