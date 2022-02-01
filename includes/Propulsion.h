/**
 * @file        Propulsion.h
 * @authors     Ayberk Yaraneri
 *
 * @brief       Class definitions for rocket propulsion components
 *
 * The classes defined here represent rocket propulsion components. While these
 * objects will usually represent rocket motors, they can also be defined to
 * represent components like gas thrusters or other force enducing mechanisms.
 *
 */

#pragma once

#ifndef _PROPULSION_H_
#define _PROPULSION_H_

#include <Eigen/Dense>
#include <string>
#include <vector>

using Eigen::Vector3d;

class SolidMotor {
   public:
    SolidMotor(double max_burn_duration, double thrust_value)
        : ignition_(false),
          max_burn_duration_(max_burn_duration),
          thrust_value_(thrust_value){};

    void ignite(double tStamp);
    void get_thrust(double tStamp, Vector3d& vector) const;
    double current_thrust(double tStamp) const;
    Vector3d get_thrust(double tStamp) const;

   private:
    bool ignition_;
    double max_burn_duration_;
    double ignition_tStamp_{};
    double current_thrust_{};
    double thrust_value_;
};

#endif
