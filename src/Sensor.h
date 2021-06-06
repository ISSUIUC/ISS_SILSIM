/**
 * @file 		Sensor.h	
 * @authors 	Ayberk Yaraneri
 *
 * @brief 		Class definition for sensor components
 *
 * These Sensor classes encapsulate typical sensors found on Rocket avionics 
 * like accelerometers and gyroscopes. This is the mechanism through which 
 * the flight software being tested can obtain information of the simulated rocket.
 * The classes provide a modular a means of injecting noise and bias along with
 * other inaccuracies to make sensor measurements behave closer to real hardware
 * rather than simply proving gorund truth. 
 *
 */

#pragma once

#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <string>
#include <vector>

#include "Rocket.h"
#include "Vector3.h"

class Sensor {
public:
	Sensor(Rocket& rocket, double refresh_rate) : 	_rocket(rocket),
													_refresh_rate(refresh_rate),
													_last_update_tStep(0),
													_data_scalar(0) {};

	bool is_new_data() {return _new_data;};

	virtual void update_data(double tStep) = 0;
	virtual void get_data(Vector3& data) = 0;

protected:
	Rocket& _rocket;

	double _refresh_rate;
	double _last_update_tStep;
	bool _new_data;

	Vector3 _data;
	double _data_scalar;
};

class Gyroscope : public Sensor {
public:
	Gyroscope(Rocket& rocket, double refresh_rate);
	void update_data(double tStep);
	void get_data(Vector3& data);
};

class Accelerometer : public Sensor {
public:
	Accelerometer(Rocket& rocket, double refresh_rate);
	void update_data(double tStep);
	void get_data(Vector3& data);
};

class Barometer : public Sensor {
public:
	Barometer(Rocket& rocket, double refresh_rate);
	void update_data(double tStep);
	void get_data(double& data);
};

#endif
