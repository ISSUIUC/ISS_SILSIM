#include <string>
#include <vector>
#include <math.h>

#include "Engine.h"
#include "Rocket.h"
#include "quaternion.h"
#include "Vector3.h"

void ForwardEuler::march_step(double tStamp, double tStep)
{
	static Vector3 f_net = _rocket.get_f_net();
	static Vector3 r_vect = _rocket.get_r_vect();
	static Vector3 r_dot = _rocket.get_r_dot();
	static Vector3 r_ddot = _rocket.get_r_ddot();
	static double Cd = _rocket.get_Cd();
	static double mass = _rocket.get_mass();

	static Vector3 thrust = _motor.get_thrust(tStamp);

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
