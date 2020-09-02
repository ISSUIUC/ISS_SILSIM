#ifndef _PROPULSION_H_
#define _PROPULSION_H_

#include <string>
#include <vector>

#include "Vector3.h"

class SolidMotor {
public:
	SolidMotor(double max_burn_duration, double thrust_value):
		_ignition(false),
		_max_burn_duration(max_burn_duration),
		_thrust_value(thrust_value) {};

	void ignite(double tStamp);
	void get_thrust(double tStamp, Vector3& vector);

private:
	bool _ignition;
	double _max_burn_duration;
	double _ignition_tStamp;
	double _current_thrust;
	double _thrust_value;
};

#endif
