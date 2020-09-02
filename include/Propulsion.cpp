#include <string>
#include <vector>
#include <iostream>

#include "Propulsion.h"

void SolidMotor::ignite(double tStamp) {
	_ignition = true;
	_ignition_tStamp = tStamp;
	_current_thrust = _thrust_value;
}

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
