/**
 * @file 		Simulation.h
 * @authors 	Ayberk Yaraneri
 *
 * @brief 		Simulation class definition 
 *
 * The Simulation class encapsulates meta information about the simulation.
 * The class contains references to relevant components of the simulation to
 * be performed, like a PhysicsEngine object and a Rocket object. It also 
 * incorporates mechanisms for data logging and diagnostics/debugging.
 *
 * In the hierearchy of SILSIM, the Simulation class sits at the top of the
 * tree and "governs" the simulation itself. 
 *
 */

#pragma once

#ifndef _SIMULATION_H_
#define _SIMULATION_H_

#include <string>
#include <fstream>

#include "PhysicsEngine.h"
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
