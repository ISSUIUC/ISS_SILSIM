/**
 * @file 		Sensor.h	
 * @authors 	Ayberk Yaraneri
 *
 * @brief 		Class definition for Sensor classes 
 *
 * These Sensor classes encapsulate typical sensors found on Rocket avionics 
 * like accelerometers and gyroscopes. This is the mechanism through which 
 * the flight software being tested can obtain information of the simulated rocket.
 * The classes provide a modular a means of injecting noise and bias along with
 * other inaccuracies to make sensor measurements behave closer to real hardware
 * rather than simply providing gorund truth. 
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
		Sensor(std::string name, Rocket& rocket, double refresh_rate) :
														_name(name),
														_rocket(rocket),
														_refresh_rate(refresh_rate),
														_last_update_tStep(0) {};

		bool is_new_data() {return _new_data;};

		virtual void update_data(double tStep) = 0;
		virtual void get_data(Vector3& data) = 0;
		// virtual void get_data(double& data) = 0;

	protected:

		std::string _name;
		
		Rocket& _rocket;

		double _refresh_rate;
		double _last_update_tStep;
		bool _new_data;

};

class Gyroscope : public Sensor {
	public:
		Gyroscope(std::string name, Rocket& rocket, double refresh_rate);
		void update_data(double tStep);
		void get_data(Vector3& data);

	private:
		Vector3 _data; // The sensor's current reading

		bool _inject_noise; 		// Whether to inject random noise to measurement
		double _noise_magnitude; 	// Maximum magnitude of the noise
		double _noise_std_dev; 		// Standard deviation of the noise
		Vector3 _noise; 			// Noise vector to be added to measurement

		bool _inject_bias; 	// Whether to inject constant bias to measurement
		Vector3 _bias; 		// Constant bias vector to be added to measurement	
};

class Accelerometer : public Sensor {
	public:
		Accelerometer(std::string name, Rocket& rocket, double refresh_rate);
		void update_data(double tStep);
		void get_data(Vector3& data);

 	private:
		Vector3 _data; // The sensor's current reading

		bool _inject_noise; 		// Whether to inject random noise to measurement
		double _noise_magnitude; 	// Maximum magnitude of the noise
		double _noise_std_dev; 		// Standard deviation of the noise
		Vector3 _noise; 			// Noise vector to be added to measurement

		bool _inject_bias; 	// Whether to inject constant bias to measurement
		Vector3 _bias; 		// Constant bias vector to be added to measurement	
};

// TODO: Implement functions for both altitude and pressure measurements (and specify units)
class Barometer : public Sensor {
	public:
		Barometer(std::string name, Rocket& rocket, double refresh_rate);
		void update_data(double tStep);
		void get_data(double& data);
	
	private:
		double _data; 	// The sensor's current reading

		bool _inject_noise; 		// Whether to inject random noise to measurement
		double _noise_magnitude; 	// Maximum magnitude of the noise
		double _noise_std_dev; 		// Standard deviation of the noise
		double _noise; 				// Noise value to be added to measurement

		bool _inject_bias; 	// Whether to inject constant bias to measurement
		double _bias; 		// Constant bias value to be added to measurement	
};

#endif
