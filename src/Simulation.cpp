/**
 * @file 		Simulation.cpp
 * @authors 	Ayberk Yaraneri
 *
 * @brief 		Simulation class member function implementations 
 *
 * The Simulation class encapsulates meta information about the simulation.
 * It also contains references to relevant components of the simulation to
 * be performed, like a PhysicsEngine object and a Rocket object.
 *
 * In the hierearchy of SILSIM, the Simulation class sits at the top of the
 * tree and "governs" the simulation itself. 
 *
 */

#include <iostream>
#include <string>
#include <fstream>
#include <math.h>

#include "Simulation.h"
#include "PhysicsEngine.h"
#include "Rocket.h"
#include "Propulsion.h"
#include "Sensor.h"
#include "quaternion.h"
#include "Vector3.h"

#define RAD2DEG (180.0 / 3.14159265);

// #define SIM_DEBUG

void Simulation::run(int steps) {

	std::ofstream dataFile(_filename);

	Vector3 r_vect;
	Vector3 r_dot;
	Vector3 r_ddot;
	Vector3 f_net;
	Vector3 w_net;

	Quaternion<double> q_ornt;

	double roll,pitch,yaw;

	_motor.ignite(_tStamp);

	for (int iter = 0; iter < steps; ++iter) {

		_rocket.get_r_vect(r_vect);
		_rocket.get_r_dot(r_dot);
		_rocket.get_r_ddot(r_ddot);
		_rocket.get_f_net(f_net);
		_rocket.get_w_vect(w_net);
		_rocket.get_q_ornt(q_ornt);

		float s = q_ornt.Gets();
		float x = q_ornt.Getx();
		float y = q_ornt.Gety();
		float z = q_ornt.Getz();

		// eqns from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
		yaw  = atan2(2.0*(s*x + z*y), 1.0 - 2.0*(x*x + y*y)) * RAD2DEG;
		pitch = asin(2.0*(s*y - z*x)) * RAD2DEG;
		roll = atan2(2.0*(s*z + x*y), -1.0 + 2.0*(s*s + x*x)) * RAD2DEG;


#ifdef SIM_DEBUG
		double alpha = acos(_rocket.i2r(r_dot).z / (r_dot.magnitude()));
		printf("Timestamp: %f\n", _tStamp);
		printf("\tR-Vector: <%f, %f, %f>", r_vect.x, r_vect.y, r_vect.z);
		printf("\tVelocity: <%f, %f, %f>", r_dot.x, r_dot.y, r_dot.z);
		printf("\tAccel: <%f, %f, %f>", r_ddot.x, r_ddot.y, r_ddot.z);
		printf("\tF-Net: <%f, %f, %f>", f_net.x, f_net.y, f_net.z);
		printf("\tW-Net: <%f, %f, %f>\n", w_net.x, w_net.y, w_net.z);
		printf("ROLL: %f \tPITCH: %f \tYAW: %f  [deg]", roll, pitch, yaw);
		printf("\nalphaSIM: %f  [deg]\n\n", alpha*RAD2DEG);
#endif

		Vector3 rocket_axis(0, 0, 1);
		rocket_axis = _rocket.r2i(rocket_axis);

		_engine.march_step(_tStamp, _tStep);

		update_sensors();

		dataFile << _tStamp << ",";
		dataFile << r_vect.x << "," << r_vect.y << "," << r_vect.z << ",";
		dataFile << r_dot.x << "," << r_dot.y << "," << r_dot.z << ",";
		dataFile << r_ddot.x << "," << r_ddot.y << "," << r_ddot.z << ",";
		dataFile << f_net.x << "," << f_net.y << "," << f_net.z << ",";
		dataFile << s << "," << x << "," << y << "," << z  << ",";
		dataFile << roll << "," << pitch << "," << yaw << ",";
		dataFile << rocket_axis.x << "," << rocket_axis.y << "," << rocket_axis.z << ",";
		
		Vector3 sensor_data;
		_sensors[0]->get_data(sensor_data);
		dataFile << sensor_data.x << "," << sensor_data.y << "," << sensor_data.z;

		dataFile << "\n";

		_tStamp += _tStep;

		if (r_dot.z < -3.0) {
			break;
		}
	}

	dataFile.close();
}

/**
 * @brief Adds a new sensor on the rocket to the simulation 
 *
 * @param sensor A pointer to the new Sensor object to be added
 */
void Simulation::add_sensor(Sensor* sensor) {
		_sensors.push_back(sensor);
}

/**
 * @brief Updates all sensors' internal data 
 *
 */
void Simulation::update_sensors() {
		for (std::vector<Sensor*>::iterator it = _sensors.begin(); it != _sensors.end(); ++it) {
				(*it)->update_data(_tStamp);
		}

}
