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
#define _USE_MATH_DEFINES

#include <Eigen/Dense>
#include <array>
#include <string>
#include <vector>

using Eigen::Vector3d;

using Eigen::Quaterniond;

class Rocket {
   public:
    Rocket() {
        q_ornt_ = {1, 0, 0, 0};

        Cp_vect_ = {0, 0, -(nose_to_cp_ - nose_to_cg_)};
    }

    /************ Get parameters ***************/
    Vector3d get_r_vect() const { return r_vect_; };
    Vector3d get_r_dot() const { return r_dot_; };
    Vector3d get_r_ddot() const { return r_ddot_; };

    Quaterniond get_q_ornt() const { return q_ornt_; };

    Vector3d get_w_vect() const { return w_vect_; };
    Vector3d get_w_dot() const { return w_dot_; };

    Vector3d get_f_net() const { return f_net_; };
    Vector3d get_t_net() const { return t_net_; };

    double get_structural_mass() const { return structural_mass_; };
    double get_total_mass() const { return total_mass_; };
    double get_d_ref() const { return d_ref_; };
    double get_A_ref() const { return A_ref_; };
    double get_Cna() const { return Cna_; };
    double get_Cd() const { return Cd_; };
    double get_nose_to_cg() const { return nose_to_cg_; };
    double get_nose_to_cp() const { return nose_to_cp_; };

    Vector3d get_Cp_vect() const { return Cp_vect_; };

    Vector3d get_launch_ecef() const { return launch_ecef_; };
    Vector3d get_launch_geod() const { return launch_geod_; };

    std::array<double, 9> get_I() const { return I_; };

    /************* Set parameters ***************/
    void set_r_vect(Vector3d vector) { r_vect_ = vector; };
    void set_r_dot(Vector3d vector) { r_dot_ = vector; };
    void set_r_ddot(Vector3d vector) { r_ddot_ = vector; };

    void set_q_ornt(Quaterniond quatrn) { q_ornt_ = quatrn; };

    void set_I(const std::array<double, 9>& array) { I_ = array; };

    void set_w_vect(Vector3d vector) { w_vect_ = vector; };
    void set_w_dot(Vector3d vector) { w_dot_ = vector; };

    void set_f_net(Vector3d vector) { f_net_ = vector; };
    void set_t_net(Vector3d vector) { t_net_ = vector; };

    void set_structural_mass(double mass) { structural_mass_ = mass; };
    void set_total_mass(double mass) { total_mass_ = mass; };
    void set_d_ref(double d_ref) { d_ref_ = d_ref; };
    void set_A_ref(double A_ref) { A_ref_ = A_ref; };
    void set_Cna(double Cna) { Cna_ = Cna; };
    void set_Cd(double Cd) { Cd_ = Cd; };
    void set_nose_to_cg(double nose_to_cg) {
        nose_to_cg_ = nose_to_cg;
        Cp_vect_ = {0, 0, -(nose_to_cp_ - nose_to_cg_)};
    };
    void set_nose_to_cp(double nose_to_cp) {
        nose_to_cp_ = nose_to_cp;
        Cp_vect_ = {0, 0, -(nose_to_cp_ - nose_to_cg_)};
    };

    // Converts vector from ENU frame to rocket reference frame
    Vector3d enu2r(Vector3d vector);

    // Converts vector from rocket frame to ENU reference frame
    Vector3d r2enu(Vector3d vector);

    // Converts vector from ENU frame to ECEF reference frame
    Vector3d enu2ecef(Vector3d pos_enu);

    // Converts vector from ECEF frame to Geodetic reference frame
    Vector3d ecef2geod(Vector3d ecef);

   private:
    // The following are in ENU frame
    Vector3d r_vect_{0, 0, 0};  // r vector
    Vector3d r_dot_{0, 0, 0};   // r-dot (velocity)
    Vector3d r_ddot_{0, 0, 0};  // r-double-dot (acceleration)
    Vector3d w_vect_{0, 0, 0};  // angular velocity (omega) vector
    Vector3d w_dot_{0, 0, 0};   // angular acceleration vector

    // The following are in ENU frame
    Vector3d f_net_{0, 0, 0};  // net force in Netwons
    Vector3d t_net_{0, 0, 0};  // net torque in Newton*meters

    Quaterniond q_ornt_{};  // ENU -> rocket frame quaternion

    // The following are in rocket frame
    Vector3d Cp_vect_{};  // CG to Cp vector

    std::array<double, 9> I_{};  // Rocket moment of inertia tensor

    // The following are in Geocentric frame
    Vector3d launch_ecef_{150992.99, -4882549.85, 4087626.55};
    Vector3d launch_geod_{
        40.111801, -88.228691,
        216};  // (40.111801, -88.228691, 216) - Talbot Laboratory

    // Default scalar parameters from OpenRocket
    double structural_mass_ = 41.034;  // structural mass of rocket in Kg
    double total_mass_ = 41.034;       // total mass including propellant in Kg
    double d_ref_ = 0.0157;            // ref length in m
    double A_ref_ = 0.0194;            // ref area in m^2
    double Cna_ = 9.65;                // normal force coefficient derivative
                                       // wrt angle-of-attack
    double Cd_ = 0.630;                // drag coefficient
    double nose_to_cg_ = 3.59;         // nosecone tip to CG distance in m
    double nose_to_cp_ = 4.03;         // nosecone tip to Cp distance in m
};

#endif
