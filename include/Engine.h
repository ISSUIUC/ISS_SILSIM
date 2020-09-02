
#ifndef _ENGINE_H_
#define _ENGINE_H_

#include <string>
#include <vector>

#include "Rocket.h"
#include "Propulsion.h"
#include "quaternion.h"

class PhysicsEngine {
public:
	PhysicsEngine(Rocket& rocket, SolidMotor& motor) :
		_rocket(rocket), _motor(motor) {};

	virtual void march_step(double tStamp, double tStep) = 0;

protected:
	Rocket& _rocket;
	SolidMotor& _motor;
};


class ForwardEuler : public PhysicsEngine {
public:
	ForwardEuler(Rocket& rocket, SolidMotor& motor) :
		PhysicsEngine(rocket, motor) {};

	void march_step(double tStamp, double tStep);
};

#endif
