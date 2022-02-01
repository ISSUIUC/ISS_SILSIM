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

#include<rapidcsv.h>

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
            vector.z() = current_thrust(tStamp);
            return;
        }
    }
    vector.x() = 0.0;
    vector.y() = 0.0;
    vector.z() = 0.0;
}

/**
 * @brief Returns current thrust of rocket
 *
 * @param tStamp Current simulation timestamp
 * 
 */
double SolidMotor::current_thrust(double tStamp) const {
    
    rapidcsv::Document csv("thrust_data/data.csv");     // Imports thrust data for Cesaroni 
    auto time = csv.GetColumn<double>("Time");
    auto thrust = csv.GetColumn<double>("Thrust");
    
    int n_data = time.size();
    
    for (int i = 1; i < n_data; i++) {                  // Returns different thrust values based on time
        if (tStamp < time[i]) {
            double slope =(thrust[i] - thrust[i - 1]) / (time[i] - time[i - 1]);
            return slope*(tStamp - time[i - 1]) + thrust[i - 1];
        }
    }

    return 0;

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
