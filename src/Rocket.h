/**
 * @file 		Rocket.h
 * @authors 	Ayberk Yaraneri
 *
 * @brief 		Rocket class definition
 *
 * The rocket class encapsulates all physical quantities and parameters of the
 * Rocket. The class also contains references to child objects representing
 * components like sensors and rocket motors along with mechanisms to extract
 * useful information about the rocket's state/trajectory.
 *
 */

#pragma once

#ifndef _ROCKET_H_
#define _ROCKET_H_

#include <string>
#include <vector>

#include "Vector3.h"
#include "quaternion.h"

class Rocket {
   public:
    Rocket();

    /****************** Get parameters by referece ************************/
    void get_r_vect(Vector3& vector) const { vector = _r_vect; };
    void get_r_dot(Vector3& vector) const { vector = _r_dot; };
    void get_r_ddot(Vector3& vector) const { vector = _r_ddot; };

    void get_q_ornt(Quaternion<double>& quatrn) const { quatrn = _q_ornt; };

    void get_I(double (&array)[9]) const {
        for (int i = 0; i < 9; ++i) {
            array[i] = _I[i];
        }
    };

    void get_w_vect(Vector3& vector) const { vector = _w_vect; };
    void get_w_dot(Vector3& vector) const { vector = _w_dot; };

    void get_f_net(Vector3& vector) const { vector = _f_net; };
    void get_t_net(Vector3& vector) const { vector = _t_net; };

    void get_mass(double& mass) const { mass = _mass; };
    void get_d_ref(double& d_ref) const { d_ref = _d_ref; };
    void get_A_ref(double& A_ref) const { A_ref = _A_ref; };
    void get_Cna(double& Cna) const { Cna = _Cna; };
    void get_Cd(double& Cd) const { Cd = _Cd; };
    void get_nose_to_cg(double& nose_to_cg) const;
    void get_nose_to_cp(double& nose_to_cp) const;

    void get_Cp_vect(Vector3& vector) const;

    /************ Get parameters by value (return by value) ***************/
    Vector3 get_r_vect() const { return _r_vect; };
    Vector3 get_r_dot() const { return _r_dot; };
    Vector3 get_r_ddot() const { return _r_ddot; };

    Quaternion<double> get_q_ornt() const { return _q_ornt; };

    Vector3 get_w_vect() const { return _w_vect; };
    Vector3 get_w_dot() const { return _w_dot; };

    Vector3 get_f_net() const { return _f_net; };
    Vector3 get_t_net() const { return _t_net; };

    double get_mass() const { return _mass; };
    double get_d_ref() const { return _d_ref; };
    double get_A_ref() const { return _A_ref; };
    double get_Cna() const { return _Cna; };
    double get_Cd() const { return _Cd; };
    double get_nose_to_cg() const { return _nose_to_cg; };
    double get_nose_to_cp() const { return _nose_to_cp; };

    Vector3 get_Cp_vect() const { return _Cp_vect; };

    /************* Set parameters (all passed by reference) ***************/
    void set_r_vect(Vector3& vector) { _r_vect = vector; };
    void set_r_dot(Vector3& vector) { _r_dot = vector; };
    void set_r_ddot(Vector3& vector) { _r_ddot = vector; };

    void set_q_ornt(Quaternion<double>& quatrn) { _q_ornt = quatrn; };

    void set_I(double (&array)[9]) {
        for (int i = 0; i < 9; ++i) {
            _I[i] = array[i];
        }
    };

    void set_w_vect(Vector3& vector) { _w_vect = vector; };
    void set_w_dot(Vector3& vector) { _w_dot = vector; };

    void set_f_net(Vector3& vector) { _f_net = vector; };
    void set_t_net(Vector3& vector) { _t_net = vector; };

    void set_mass(double& mass) { _mass = mass; };
    void set_d_ref(double& d_ref) { _d_ref = d_ref; };
    void set_A_ref(double& A_ref) { _A_ref = A_ref; };
    void set_Cna(double& Cna) { _Cna = Cna; };
    void set_Cd(double& Cd) { _Cd = Cd; };
    void set_nose_to_cg(double& nose_to_cg);
    void set_nose_to_cp(double& nose_to_cp);

    // Converts vector from inertial frame to rocket reference frame
    Vector3 i2r(Vector3 vector);

    // Converts vector from rocket frame to inertial reference frame
    Vector3 r2i(Vector3 vector);

   private:
    // The following are in inertial frame
    Vector3 _r_vect;  // r vector
    Vector3 _r_dot;   // r-dot (velocity)
    Vector3 _r_ddot;  // r-double-dot (acceleration)
    Vector3 _w_vect;  // angular velocity (omega) vector
    Vector3 _w_dot;   // angular acceleration vector

    // The following are in inertial frame
    Vector3 _f_net;  // net force in Netwons
    Vector3 _t_net;  // net torque in Newton*meters

    Quaternion<double> _q_ornt;  // inertial -> rocket frame quaternion

    // The following are in rocket frame
    Vector3 _Cp_vect;  // CG to Cp vector

    double _I[9];  // Rocket moment of inertia tensor

    // Default scalar parameters from OpenRocket
    double _mass = 41.034;      // in Kg
    double _d_ref = 0.0157;     // ref length in m
    double _A_ref = 0.0194;     // ref area in m^2
    double _Cna = 9.65;         // normal force coefficient derivative
                                // wrt angle-of-attack
    double _Cd = 0.630;         // drag coefficient
    double _nose_to_cg = 3.59;  // nosecone tip to CG distance in m
    double _nose_to_cp = 4.03;  // nosecone tip to Cp distance in m
};

#endif
