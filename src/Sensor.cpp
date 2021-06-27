/**
 * @file 		Sensor.cpp	
 * @authors 	Ayberk Yaraneri
 *
 * @brief 		Member function implementations for Sensor classes	
 *
 * These Sensor classes encapsulate typical sensors found on Rocket avionics 
 * like accelerometers and gyroscopes. This is the mechanism through which 
 * the flight software being tested can obtain information of the simulated rocket.
 * The classes provide a modular a means of injecting noise and bias along with
 * other inaccuracies to make sensor measurements behave closer to real hardware
 * rather than simply providing gorund truth. 
 *
 */

#include <vector>
#include <string>

#include "Rocket.h"
#include "Sensor.h"

Gyroscope::Gyroscope(std::string name, Rocket& rocket, double refresh_rate) :
								Sensor(name, rocket, refresh_rate) {
	_data = Vector3();
}

void Gyroscope::update_data(double tStep) {
	if ((tStep - _last_update_tStep) >= (1 / _refresh_rate)) {
		_rocket.get_w_vect(_data);
		_rocket.i2r(_data);
		_new_data = true;
	}
}

void Gyroscope::get_data(Vector3& data) {
	data = _data;
	_new_data = false;
}

Accelerometer::Accelerometer(std::string name, Rocket& rocket, double refresh_rate) :
								Sensor(name, rocket, refresh_rate) {
	_data = Vector3();
}

void Accelerometer::update_data(double tStep) {
	if ((tStep - _last_update_tStep) >= (1 / _refresh_rate)) {
		_rocket.get_r_ddot(_data);
		_rocket.i2r(_data);
		_new_data = true;
	}
}

void Accelerometer::get_data(Vector3& data) {
	data = _data;
	_new_data = false;
}

void Barometer::update_data(double tStep) {
	if ((tStep - _last_update_tStep) >= (1 / _refresh_rate)) {
		_data = _rocket.get_r_vect().x;
		_new_data = true;
	}
}

void Barometer::get_data(double& data) {
	data = _data;
	_new_data = false;
}
