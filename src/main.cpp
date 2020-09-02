#include <iostream>
#include <string>
#include <stdio.h>

#include "Vector3.h"
#include "Simulation.h"
#include "Rocket.h"
#include "Sensor.h"
#include "quaternion.h"

int main() {

	Rocket rocket;

	// 3.5 second burn time @ 1500 Newton constant thrust (L ish motor I think)
	SolidMotor motor(3.5, 1500.0);

	ForwardEuler engine(rocket, motor);

	// std::vector<Sensor&> sensors;

	Simulation sim(0.01, engine, rocket, motor, "sim_data/data.csv");

	std::cout << "Running Sim!" << std::endl;

	// run 3000 steps
	sim.run(3000);

	return 0;
}
