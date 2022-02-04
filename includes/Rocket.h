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

#include <Eigen/Dense>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "RASAeroImport.h"

using Eigen::Vector3d;

using Eigen::Quaterniond;

class Rocket {
   public:
    Rocket() {
        q_ornt_ = {1, 0, 0, 0};

        cp_vect_ = {0, 0, -(nose_to_cp_ - nose_to_cg_)};
    }

    /*************************** Get parameters *****************************#*/
    Vector3d get_r_vect() const { return r_vect_; };
    Vector3d get_r_dot() const { return r_dot_; };
    Vector3d get_r_ddot() const { return r_ddot_; };

    Quaterniond get_q_ornt() const { return q_ornt_; };

    Vector3d get_w_vect() const { return w_vect_; };
    Vector3d get_w_dot() const { return w_dot_; };

    Vector3d get_f_net() const { return f_net_; };
    Vector3d get_t_net() const { return t_net_; };

    double get_mass() const { return mass_; };
    double get_reference_length() const { return reference_length_; };
    double get_reference_area() const { return reference_area_; };
    double get_total_normal_force_coeff() const {
        return total_normal_force_coeff_;
    };
    double get_total_axial_force_coeff() const {
        return total_axial_force_coeff_;
    };
    double get_nose_to_cg() const { return nose_to_cg_; };
    double get_nose_to_cp() const { return nose_to_cp_; };

    Vector3d get_cp_vect() const { return cp_vect_; };

    std::array<double, 9> get_I() const { return I_; };

    /**************************** Set parameters ******************************/
    void set_r_vect(Vector3d vector) { r_vect_ = vector; };
    void set_r_dot(Vector3d vector) { r_dot_ = vector; };
    void set_r_ddot(Vector3d vector) { r_ddot_ = vector; };

    void set_q_ornt(Quaterniond quatrn) { q_ornt_ = quatrn; };

    void set_I(const std::array<double, 9>& array) { I_ = array; };

    void set_w_vect(Vector3d vector) { w_vect_ = vector; };
    void set_w_dot(Vector3d vector) { w_dot_ = vector; };

    void set_f_net(Vector3d vector) { f_net_ = vector; };
    void set_t_net(Vector3d vector) { t_net_ = vector; };

    void set_mass(double mass) { mass_ = mass; };
    void set_reference_length(double d_ref) { reference_length_ = d_ref; };
    void set_reference_area(double a_ref) { reference_area_ = a_ref; };
    void set_total_normal_force_coeff(double cn_total) {
        total_normal_force_coeff_ = cn_total;
    };
    void set_total_axial_focre_coeff(double ca_total) {
        total_axial_force_coeff_ = ca_total;
    };
    void set_nose_to_cg(double nose_to_cg) {
        nose_to_cg_ = nose_to_cg;
        cp_vect_ = {0, 0, -(nose_to_cp_ - nose_to_cg_)};
    };
    void set_nose_to_cp(double nose_to_cp) {
        nose_to_cp_ = nose_to_cp;
        cp_vect_ = {0, 0, -(nose_to_cp_ - nose_to_cg_)};
    };

    /************************ Internal State Update ***************************/
    void update_aero_coefficients(bool poweron, double protuberance_perecent);

    /*************************** Helper Functions  ****************************/
    // Converts vector from inertial frame to rocket reference frame
    Vector3d i2r(Vector3d vector);

    // Converts vector from rocket frame to inertial reference frame
    Vector3d r2i(Vector3d vector);


   private:
    // The following are in inertial frame
    Vector3d r_vect_{0, 0, 0};  // r vector
    Vector3d r_dot_{0, 0, 0};   // r-dot (velocity)
    Vector3d r_ddot_{0, 0, 0};  // r-double-dot (acceleration)
    Vector3d w_vect_{0, 0, 0};  // angular velocity (omega) vector
    Vector3d w_dot_{0, 0, 0};   // angular acceleration vector

    // The following are in inertial frame
    Vector3d f_net_{0, 0, 0};  // net force in Netwons
    Vector3d t_net_{0, 0, 0};  // net torque in Newton*meters

    Quaterniond q_ornt_{};  // inertial -> rocket frame quaternion

    Vector3d cp_vect_{};  // CG to CP vector, rocket frame

    double mach_ = 0.0;   // Freestream air mach number;
    double alpha_ = 0.0;  // Rocket total angle-of-attack to air

    //---------- Intertial Parameters ----------
    std::array<double, 9> I_{};  // Rocket moment of inertia tensor
    double mass_ = 41.034;       // in Kg
    double nose_to_cg_ = 3.59;   // nosecone tip to CG distance in m

    //----------- Aerodynamic Parameters ----------
    std::shared_ptr<RASAeroImport> rasaero_import_;  // Aero lookup table
    double reference_length_ = 0.0157;               // reference length in m
    double reference_area_ = 0.0194;                 // reference area in m^2
    double total_normal_force_coeff_ = 9.65;  // total normal force coefficient
    double total_axial_force_coeff_ = 0.630;  // total axial force coefficient
    double nose_to_cp_ = 4.03;  // nosecone tip to Cp distance in m
};

#endif
