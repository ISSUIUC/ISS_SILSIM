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

void Simulation::run(int steps) {

	std::ofstream dataFile(_filename);

	Vector3 r_vect;
	Vector3 r_dot;
	Vector3 r_ddot;
	Vector3 f_net;

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
		printf("\tF-Net: <%f, %f, %f>\n", f_net.x, f_net.y, f_net.z);

		_engine.march_step(_tStamp, _tStep);

		dataFile << _tStamp << ",";
		dataFile << r_vect.x << "," << r_vect.y << "," << r_vect.z << ",";
		dataFile << r_dot.x << "," << r_dot.y << "," << r_dot.z << ",";
		dataFile << r_ddot.x << "," << r_ddot.y << "," << r_ddot.z << ",";
		dataFile << f_net.x << "," << f_net.y << "," << f_net.z << "\n";

		_tStamp += _tStep;
	}

	dataFile.close();
}
