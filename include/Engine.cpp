#include <string>
#include <vector>
#include <math.h>

#include "Engine.h"
#include "Rocket.h"
#include "quaternion.h"
#include "Vector3.h"

static double rad2deg = 180.0 / 3.14159265;

void ForwardEuler::march_step(double tStamp, double tStep)
{

	/* ##### Retrieve instantaneous rocket parameters ##### */

	// Inertial frame dynamics parameters
	static Vector3 r_vect_if = _rocket.get_r_vect();
	static Vector3 r_dot_if = _rocket.get_r_dot();
	static Vector3 r_ddot_if = _rocket.get_r_ddot();
	static Vector3 w_vect_if = _rocket.get_w_vect();
	static Vector3 w_dot_if = _rocket.get_w_dot();
	static Vector3 f_net_if = _rocket.get_f_net();
	static Vector3 t_net_if = _rocket.get_t_net();

	// Quaternion from inertial to rocket frame
	static Quaternion<double> q_ornt = _rocket.get_q_ornt();

	// CG to Cp vector
	static Vector3 Cp_vect_rf = _rocket.get_Cp_vect();

	// Get moment of inertia tenspr
	static double I_tens[9];
	_rocket.get_I(I_tens);

	// Static parameters
	static double mass = _rocket.get_mass();
	// static double d_ref = _rocket.get_d_ref();
	static double A_ref = _rocket.get_A_ref();
	static double Cna = _rocket.get_Cna();
	static double Cd = _rocket.get_Cd();

	// Motor thrust vector, rocket frame
	static Vector3 thrust_rf = _motor.get_thrust(tStamp);

	/* ##### Calculate forces and torques ##### */

	// printf("\n\n");

	static Vector3 f_aero_rf;	// Aerodynamic forces, rocket frame
	static Vector3 t_aero_rf;	// Aerodynamic torques, rocket frame
	static Vector3 f_aero_if;	// Aerodynamic forces, inertial frame
	static Vector3 t_aero_if;	// Aerodynamic torques, inertial frame

	static Vector3 f_net_rf;
	static Vector3 t_net_rf;

	if (r_dot_if.magnitude() > 0.1) {

		Vector3 rocket_axis_rf(0,0,1);

		Vector3 v_rf;
		v_rf = _rocket.i2r(r_dot_if);

		// printf("v_rf: <%f, %f, %f>\n", v_rf.x, v_rf.y, v_rf.z);

		double alpha = acos((rocket_axis_rf.dot(v_rf)) / (v_rf.magnitude()));
		// alpha *= rad2deg;

		printf("\nalpha: %f\n\n", alpha*rad2deg);

		Vector3 f_N_rf;

		double f_N_mag = Cna * alpha * 0.5 * 1.225 * v_rf.magnitude2() * A_ref;
		f_N_rf.x = -v_rf.x;
		f_N_rf.y = -v_rf.y;
		f_N_rf.z = 0;
		f_N_rf.normalize();
		f_N_rf = f_N_rf * f_N_mag;

		// printf("f_N_mag: %f\n", f_N_mag);
		// printf("f_N_rf: <%f, %f, %f>\n", f_N_rf.x, f_N_rf.y, f_N_rf.z);

		double f_D_mag = Cd * 0.5 * 1.225 * v_rf.magnitude2();
		Vector3 f_D_rf(0, 0, -f_D_mag);

		// printf("f_D_rf: <%f, %f, %f>\n", f_D_rf.x, f_D_rf.y, f_D_rf.z);

		f_aero_rf = f_N_rf + f_D_rf;

		// printf("f_aero_rf: <%f, %f, %f>\n\n", f_aero_rf.x, f_aero_rf.y, f_aero_rf.z);

		// t_aero_rf = f_aero_rf.cross(Cp_vect_rf);
		t_aero_rf = Cp_vect_rf.cross(f_aero_rf);

	}
	else {
		f_aero_rf.x = 0;
		f_aero_rf.y = 0;
		f_aero_rf.z = 0;

		t_aero_rf.x = 0;
		t_aero_rf.y = 0;
		t_aero_rf.z = 0;
	}

	f_net_if = _rocket.r2i(f_aero_rf + thrust_rf);
	f_net_if.z -= mass * 9.81;

	t_net_if = _rocket.r2i(t_aero_rf);

	/* ##### Perform euler step ##### */

	r_vect_if += r_dot_if * tStep;
	r_dot_if += r_ddot_if * tStep;
	r_ddot_if = f_net_if / mass;

	// Assemble instantaneous rotation quaternion
    double w_mag = w_vect_if.magnitude();
	if (w_mag > 0.000001) {
		Quaternion<double> q_rot;
	    q_rot.Set(cos(w_mag/2.0),
				  (w_vect_if.x/w_mag)*sin(w_mag/2.0),
				  (w_vect_if.y/w_mag)*sin(w_mag/2.0),
				  (w_vect_if.z/w_mag)*sin(w_mag/2.0));

		// Apply instantaneous rotation
		q_ornt = q_ornt * q_rot;
		q_ornt.Normalize();
	}

	w_vect_if += w_dot_if * tStep;

	w_dot_if.x = t_net_if.x / I_tens[0];
	w_dot_if.y = t_net_if.y / I_tens[4];
	w_dot_if.z = t_net_if.z / I_tens[8];

	_rocket.set_f_net(f_net_if);
	_rocket.set_t_net(t_net_if);

	_rocket.set_r_vect(r_vect_if);
	_rocket.set_r_dot(r_dot_if);
	_rocket.set_r_ddot(r_ddot_if);

	_rocket.set_q_ornt(q_ornt);

	_rocket.set_w_vect(w_vect_if);
	_rocket.set_w_dot(w_dot_if);

}
