
#ifndef _ROCKET_H_
#define _ROCKET_H_

#include <string>
#include <vector>

#include "quaternion.h"
#include "Vector3.h"

class Rocket {
	public:
		Rocket();

		void get_r_vect(Vector3& vector) const;
		void get_r_dot(Vector3& vector) const;
		void get_r_ddot(Vector3& vector) const;
		void set_r_vect(Vector3& vector);
		void set_r_dot(Vector3& vector);
		void set_r_ddot(Vector3& vector);

		void get_q_ornt(Quaternion<double>& quatrn) const;
		void set_q_ornt(Quaternion<double>& quatrn);

		void get_w_vect(Vector3& vector) const;
		void get_w_dot(Vector3& vector) const;
		void set_w_vect(Vector3& vector);
		void set_w_dot(Vector3& vector);

		void get_f_net(Vector3& vector) const;
		void get_t_net(Vector3& vector) const;
		void set_f_net(Vector3& vector);
		void set_t_net(Vector3& vector);

		void get_mass(double& mass) const;
		void set_mass(double& mass);
		void get_Cd(double& Cd) const;
		void set_Cd(double& Cd);

		// Converts vector from inertial frame to rocket reference frame
		void inertial2rocket(Vector3& vector);

		// Converts vector from rocket frame to inertial reference frame
		void rocket2inertial(Vector3& vector);

	private:
		// The following are in inertial frame
		Vector3 _r_vect;	// r vector
		Vector3 _r_dot;		// r-dot (velocity)
		Vector3 _r_ddot;	// r-double-dot (acceleration)
		Vector3 _w_vect;	// angular velocity (omega) vector
		Vector3 _w_dot;		// angular acceleration vector

		// The following are in inertial frame
		Vector3 _f_net;		// net force in Netwons
		Vector3 _t_net;		// net torque in Newton*meters

		Quaternion<double> _q_ornt;		// inertial -> rocket frame quaternion

		double _mass = 20.0;			// in Kg
		double _Cd = 0.0168;			// drag coefficient

};

#endif
