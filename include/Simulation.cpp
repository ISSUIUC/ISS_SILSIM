#include <iostream>
#include <string>
#include <fstream>

#include "Simulation.h"
#include "Engine.h"
#include "Rocket.h"
#include "Propulsion.h"
#include "Sensor.h"
#include "quaternion.h"
#include "Vector3.h"

static double rad2deg = 180.0 / 3.14159265;

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

		printf("Timestamp: %f\t", _tStamp);

		_rocket.get_r_vect(r_vect);
		printf("\tR-Vector: <%f, %f, %f>", r_vect.x, r_vect.y, r_vect.z);

		_rocket.get_r_dot(r_dot);
		printf("\tVelocity: <%f, %f, %f>", r_dot.x, r_dot.y, r_dot.z);

		_rocket.get_r_ddot(r_ddot);
		printf("\tAccel: <%f, %f, %f>", r_ddot.x, r_ddot.y, r_ddot.z);

		_rocket.get_f_net(f_net);
		printf("\tF-Net: <%f, %f, %f>", f_net.x, f_net.y, f_net.z);

		_rocket.get_w_vect(w_net);
		printf("\tW-Net: <%f, %f, %f>\n", w_net.x, w_net.y, w_net.z);


		_rocket.get_q_ornt(q_ornt);

		float s = q_ornt.Gets();
		float x = q_ornt.Getx();
		float y = q_ornt.Gety();
		float z = q_ornt.Getz();

		// eqns from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
		yaw  = atan2(2.0*(s*x + z*y), 1.0 - 2.0*(x*x + y*y)) * rad2deg;
		pitch = asin(2.0*(s*y - z*x)) * rad2deg;
		roll = atan2(2.0*(s*z + x*y), -1.0 + 2.0*(s*s + x*x)) * rad2deg;

		printf("ROLL: %f\tPITCH: %f\tYAW: %f", roll, pitch, yaw);

		Vector3 rocket_axis(0, 0, 1);
		rocket_axis = _rocket.r2i(rocket_axis);

		_engine.march_step(_tStamp, _tStep);

		dataFile << _tStamp << ",";
		dataFile << r_vect.x << "," << r_vect.y << "," << r_vect.z << ",";
		dataFile << r_dot.x << "," << r_dot.y << "," << r_dot.z << ",";
		dataFile << r_ddot.x << "," << r_ddot.y << "," << r_ddot.z << ",";
		dataFile << f_net.x << "," << f_net.y << "," << f_net.z << ",";
		dataFile << s << "," << x << "," << y << "," << z  << ",";
		dataFile << roll << "," << pitch << "," << yaw << ",";
		dataFile << rocket_axis.x << "," << rocket_axis.y << "," << rocket_axis.z << "\n";

		_tStamp += _tStep;

		if (r_dot.z < -3.0) {
			break;
		}
	}

	dataFile.close();
}
