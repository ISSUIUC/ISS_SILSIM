/**
 * @file 		Propulsion.h
 * @authors 	Ayberk Yaraneri
 *
 * @brief 		Class definitions for rocket propulsion components
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
        : _ignition(false),
          _max_burn_duration(max_burn_duration),
          _thrust_value(thrust_value){};

    void ignite(double tStamp);
    void get_thrust(double tStamp, Vector3& vector);
    Vector3 get_thrust(double tStamp);

   private:
    bool _ignition;
    double _max_burn_duration;
    double _ignition_tStamp;
    double _current_thrust;
    double _thrust_value;
};

#endif
