#include <iostream>
#include <string>
#include <stdio.h>

#include "Vector3.h"

#include "Simulation.h"
#include "Rocket.h"
#include "Sensor.h"
#include "quaternion.h"

double deg2rad = 3.14159265 / 180.0;

int main() {

	Rocket rocket;

	double mass = rocket.get_mass();
	double I_tensor[9];
	I_tensor[0] = (1.0/12.0) * mass * 5.182 * 5.182 * 25;
	I_tensor[4] = I_tensor[0];
	I_tensor[8] = 0.5 * mass * 0.0762 * 0.0762;
	rocket.set_I(I_tensor);

	double angle = 5.0 * deg2rad;
	Quaternion<double> start_ornt(cos(angle/2.0), sin(angle/2.0)*0.707, sin(angle/2.0)*0.707, 0);
	rocket.set_q_ornt(start_ornt);

	// 3.5 second burn time @ 1500 Newton constant thrust (L ish motor I think)
	SolidMotor motor(3.5, 4000.0);

	ForwardEuler engine(rocket, motor);

	// std::vector<Sensor&> sensors;

	Simulation sim(0.01, engine, rocket, motor, "sim_data/data.csv");

	std::cout << "Running Sim!" << std::endl;

	// run 3000 steps
	sim.run(10000);

	return 0;
}
