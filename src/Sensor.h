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
 * rather than simply providing ground truth. 
 *
 */

#pragma once

#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <string>
#include <vector>
#include <random>

#include "Rocket.h"
#include "Vector3.h"

class Sensor {
	public:
		Sensor(std::string name, Rocket& rocket, double refresh_rate,
			   double noise_mean=0.0f, double noise_stddev=0.1f) :
														_name(name),
														_rocket(rocket),
														_refresh_rate(refresh_rate),
														_last_update_tStep(0),
														_normal_dist(noise_mean, noise_stddev) {};

		bool is_new_data() {return _new_data;};
		std::string get_name() {return _name;};

		virtual void update_data(double tStep) = 0;
		virtual void get_data(Vector3& data) = 0;
		// virtual void get_data(double& data) = 0;

		// Noise/bias injection control
		void enable_noise_injection() 	{_inject_noise = true;};
		void disable_noise_injection() 	{_inject_noise = false;};
		void enable_bias_injection() 	{_inject_bias = true;};
		void disable_bias_injection() 	{_inject_bias = false;};

	protected:

		std::string _name;
		
		Rocket& _rocket;

		double _refresh_rate;
		double _last_update_tStep;
		bool _new_data;

		// Normal distribution noise generation
		std::default_random_engine _generator;
		std::normal_distribution<double> _normal_dist;
		bool _inject_bias = false; 			// Whether to inject constant bias to measurement
		bool _inject_noise = false; 		// Whether to inject random noise to measurement
};

class Gyroscope : public Sensor {
	public:
		Gyroscope(std::string name, Rocket& rocket, double refresh_rate,
				  double noise_mean=0.0f, double noise_stddev=0.1f);
		void update_data(double tStep);
		void get_data(Vector3& data);

		void set_constant_bias(Vector3 bias) {_bias = bias;};

	private:
		Vector3 _data; 		// The sensor's current reading

		Vector3 _noise; 	// Noise vector to be added to measurement
				
		Vector3 _bias; 		// Constant bias vector to be added to measurement	
};

class Accelerometer : public Sensor {
	public:
		Accelerometer(std::string name, Rocket& rocket, double refresh_rate,
					  double noise_mean=0.0f, double noise_stddev=0.1f);
		void update_data(double tStep);
		void get_data(Vector3& data);

		void set_constant_bias(Vector3 bias) {_bias = bias;};

 	private:
		Vector3 _data; 		// The sensor's current reading

		Vector3 _noise; 	// Noise vector to be added to measurement

		Vector3 _bias; 		// Constant bias vector to be added to measurement	
};

// TODO: Implement functions for both altitude and pressure measurements (and specify units)
class Barometer : public Sensor {
	public:
		Barometer(std::string name, Rocket& rocket, double refresh_rate,
				  double noise_mean=0.0f, double noise_stddev=0.1f);
		void update_data(double tStep);
		void get_data(double& data);

		void set_constant_bias(double bias) {_bias = bias;};
	
	private:
		double _data; 		// The sensor's current reading

		double _noise; 		// Noise value to be added to measurement

		double _bias; 		// Constant bias value to be added to measurement	
};

#endif
