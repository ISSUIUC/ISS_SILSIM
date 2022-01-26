/**
 * @file        Rocket.h
 * @authors     Ayberk Yaraneri
 *
 * @brief       Rocket class definition
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

#include <array>
#include <string>
#include <vector>

#include <Vector3.h>
#include <quaternion.h>

class Rocket {
   public:
    Rocket();

    /****************** Get parameters by referece ************************/
    void get_r_vect(Vector3& vector) const { vector = r_vect_; };
    void get_r_dot(Vector3& vector) const { vector = r_dot_; };
    void get_r_ddot(Vector3& vector) const { vector = r_ddot_; };

    void get_q_ornt(Quaternion<double>& quatrn) const { quatrn = q_ornt_; };

    void get_I(double (&array)[9]) const {
        for (int i = 0; i < 9; ++i) {
            array[i] = I_[i];
        }
    };

    void get_w_vect(Vector3& vector) const { vector = w_vect_; };
    void get_w_dot(Vector3& vector) const { vector = w_dot_; };

    void get_f_net(Vector3& vector) const { vector = f_net_; };
    void get_t_net(Vector3& vector) const { vector = t_net_; };

    void get_mass(double& mass) const { mass = mass_; };
    void get_d_ref(double& d_ref) const { d_ref = d_ref_; };
    void get_A_ref(double& A_ref) const { A_ref = A_ref_; };
    void get_Cna(double& Cna) const { Cna = Cna_; };
    void get_Cd(double& Cd) const { Cd = Cd_; };
    void get_nose_to_cg(double& nose_to_cg) const;
    void get_nose_to_cp(double& nose_to_cp) const;

    void get_Cp_vect(Vector3& vector) const;

    /************ Get parameters by value (return by value) ***************/
    Vector3 get_r_vect() const { return r_vect_; };
    Vector3 get_r_dot() const { return r_dot_; };
    Vector3 get_r_ddot() const { return r_ddot_; };

    Quaternion<double> get_q_ornt() const { return q_ornt_; };

    Vector3 get_w_vect() const { return w_vect_; };
    Vector3 get_w_dot() const { return w_dot_; };

    Vector3 get_f_net() const { return f_net_; };
    Vector3 get_t_net() const { return t_net_; };

    double get_mass() const { return mass_; };
    double get_d_ref() const { return d_ref_; };
    double get_A_ref() const { return A_ref_; };
    double get_Cna() const { return Cna_; };
    double get_Cd() const { return Cd_; };
    double get_nose_to_cg() const { return nose_to_cg_; };
    double get_nose_to_cp() const { return nose_to_cp_; };

    Vector3 get_Cp_vect() const { return Cp_vect_; };

    /************* Set parameters (all passed by reference) ***************/
    void set_r_vect(Vector3& vector) { r_vect_ = vector; };
    void set_r_dot(Vector3& vector) { r_dot_ = vector; };
    void set_r_ddot(Vector3& vector) { r_ddot_ = vector; };

    void set_q_ornt(Quaternion<double>& quatrn) { q_ornt_ = quatrn; };

    void set_I(double (&array)[9]) {
        for (int i = 0; i < 9; ++i) {
            I_[i] = array[i];
        }
    };

    void set_w_vect(Vector3& vector) { w_vect_ = vector; };
    void set_w_dot(Vector3& vector) { w_dot_ = vector; };

    void set_f_net(Vector3& vector) { f_net_ = vector; };
    void set_t_net(Vector3& vector) { t_net_ = vector; };

    void set_mass(double& mass) { mass_ = mass; };
    void set_d_ref(double& d_ref) { d_ref_ = d_ref; };
    void set_A_ref(double& A_ref) { A_ref_ = A_ref; };
    void set_Cna(double& Cna) { Cna_ = Cna; };
    void set_Cd(double& Cd) { Cd_ = Cd; };
    void set_nose_to_cg(double& nose_to_cg);
    void set_nose_to_cp(double& nose_to_cp);

    // Converts vector from inertial frame to rocket reference frame
    Vector3 i2r(Vector3 vector);

    // Converts vector from rocket frame to inertial reference frame
    Vector3 r2i(Vector3 vector);

   private:
    // The following are in inertial frame
    Vector3 r_vect_;  // r vector
    Vector3 r_dot_;   // r-dot (velocity)
    Vector3 r_ddot_;  // r-double-dot (acceleration)
    Vector3 w_vect_;  // angular velocity (omega) vector
    Vector3 w_dot_;   // angular acceleration vector

    // The following are in inertial frame
    Vector3 f_net_;  // net force in Netwons
    Vector3 t_net_;  // net torque in Newton*meters

    Quaternion<double> q_ornt_;  // inertial -> rocket frame quaternion

    // The following are in rocket frame
    Vector3 Cp_vect_;  // CG to Cp vector

    std::array<double, 9> I_{};  // Rocket moment of inertia tensor

    // Default scalar parameters from OpenRocket
    double mass_ = 41.034;      // in Kg
    double d_ref_ = 0.0157;     // ref length in m
    double A_ref_ = 0.0194;     // ref area in m^2
    double Cna_ = 9.65;         // normal force coefficient derivative
                                // wrt angle-of-attack
    double Cd_ = 0.630;         // drag coefficient
    double nose_to_cg_ = 3.59;  // nosecone tip to CG distance in m
    double nose_to_cp_ = 4.03;  // nosecone tip to Cp distance in m
};

#endif
