#include <string>
#include <vector>
#include <iostream>

#include "Propulsion.h"

/**
 * @brief Transitions state of SolidMotor to ignited (producing thrust)
 *
 * @param tStamp Current simulation timestamp
 */

void SolidMotor::ignite(double tStamp) {
	_ignition = true;
	_ignition_tStamp = tStamp;
	_current_thrust = _thrust_value;
}

/**
 * @brief Thrust vector getter function (by reference)
 *
 * @param tStamp Current simulation timestamp
 * @param vector Vector reference to overwrite with motor's thrust vector
 */
void SolidMotor::get_thrust(double tStamp, Vector3& vector) {
	if (_ignition == true) {
		if ((tStamp - _ignition_tStamp) <= _max_burn_duration) {
			vector.x = 0.0;
			vector.y = 0.0;
			vector.z = _current_thrust;
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
  if (_ignition == true) {
		if ((tStamp - _ignition_tStamp) <= _max_burn_duration) {
			vector.x = 0.0;
			vector.y = 0.0;
			vector.z = _current_thrust;
			return vector;
		}
	}
	vector.x = 0.0;
	vector.y = 0.0;
	vector.z = 0.0;

  return vector;
}
