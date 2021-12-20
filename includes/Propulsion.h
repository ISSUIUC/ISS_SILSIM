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

#include <string>
#include <vector>

#include "Vector3.h"

class SolidMotor {
   public:
    SolidMotor(double max_burn_duration, double thrust_value)
        : ignition_(false),
          max_burn_duration_(max_burn_duration),
          thrust_value_(thrust_value){};

    void ignite(double tStamp);
    void get_thrust(double tStamp, Vector3& vector) const;
    Vector3 get_thrust(double tStamp) const;

   private:
    bool ignition_;
    double max_burn_duration_;
    double ignition_tStamp_{};
    double current_thrust_{};
    double thrust_value_;
};

#endif
