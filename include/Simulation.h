
#ifndef _SIMULATION_H_
#define _SIMULATION_H_

#include <string>
#include <fstream>

#include "Engine.h"
#include "Rocket.h"
#include "Propulsion.h"
#include "Sensor.h"
#include "quaternion.h"

class Simulation {
public:

	Simulation(
		double tStep,
		PhysicsEngine& engine,
		Rocket& rocket,
		SolidMotor& motor,
		std::string filename
		// std::vector<Sensor&>& sensors
	) :	_tStamp(0), _tStep(tStep), _engine(engine), _rocket(rocket),
	 	_motor(motor), _filename(filename) {};

	void run(int steps);


private:
	double _tStamp;
	double _tStep;

	PhysicsEngine& _engine;

	Rocket& _rocket;
	SolidMotor& _motor;
	// std::vector<Sensor&>& _sensors;

	std::string _filename;
};

#endif
