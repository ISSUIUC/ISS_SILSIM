#include <string>
#include <vector>
#include <math.h>

#include "Engine.h"
#include "Rocket.h"
#include "quaternion.h"
#include "Vector3.h"

void ForwardEuler::march_step(double tStamp, double tStep)
{
	Vector3 f_net;
	Vector3 r_vect;
	Vector3 r_dot;
	Vector3 r_ddot;
	Vector3 thrust;
	double Cd;
	double mass;

	_rocket.get_f_net(f_net);
	_rocket.get_r_vect(r_vect);
	_rocket.get_r_dot(r_dot);
	_rocket.get_r_ddot(r_ddot);
	_rocket.get_Cd(Cd);
	_rocket.get_mass(mass);
	_motor.get_thrust(tStamp, thrust);
	_rocket.rocket2inertial(thrust);

	// printf("\nThrust: <%f, %f, %f>\n\n", thrust[0], thrust[1], thrust[2]);

	f_net = (Cd * r_dot.normalized() * -r_dot.magnitude2()) + thrust;
	f_net.z -= mass * 9.81;

	r_vect += r_dot * tStep;

	r_dot += r_ddot * tStep;

	r_ddot = f_net / mass;

	_rocket.set_f_net(f_net);
	_rocket.set_r_vect(r_vect);
	_rocket.set_r_dot(r_dot);
	_rocket.set_r_ddot(r_ddot);

}
