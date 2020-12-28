#include <string>
#include <vector>
#include <math.h>

#include "Engine.h"
#include "Rocket.h"
#include "quaternion.h"
#include "Vector3.h"

void ForwardEuler::march_step(double tStamp, double tStep)
{

	// Rocket frame dynamics parameters
	static Vector3 r_vect = _rocket.get_r_vect();
	static Vector3 r_dot = _rocket.get_r_dot();
	static Vector3 r_ddot = _rocket.get_r_ddot();
	static Vector3 w_vect = _rocket.get_w_vect();
	static Vector3 w_dot = _rocket.get_w_dot();
	static Vector3 f_net = _rocket.get_f_net();
	static Vector3 t_net = _rocket.get_t_net();

	// Quaternion from inertial to rocket frame
	static Quaternion<double> q_ornt = _rocket.get_q_ornt();

	// CG to Cp vector
	static Vector3 _Cp_vect = _rocket.get_Cp_vect();

	// Get moment of inertia tenspr
	static double I_tens[9];
	_rocket.get_I(I_tens);

	// Static parameters
	static double mass = _rocket.get_mass();
	static double d_ref = _rocket.get_d_ref();
	static double A_ref = _rocket.get_A_ref();
	static double Cna = _rocket.get_Cna();
	static double Cd = _rocket.get_Cd();

	// Motor thrust vector
	static Vector3 thrust = _motor.get_thrust(tStamp);

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
