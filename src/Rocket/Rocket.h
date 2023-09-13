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
#include <memory>
#include <string>
#include <vector>

#include "./Aero/RASAeroImport.h"
#include "ControlSurfaces/Flaps.h"

using Eigen::Quaterniond;
using Eigen::Vector3d;

class Rocket {
   public:
    Rocket() {
        q_ornt_ = {1, 0, 0, 0};
        cp_vect_ = {0, 0, -(nose_to_cp_ - nose_to_cg_)};
    }

    Rocket(std::shared_ptr<RASAeroImport> rasaero) {
        q_ornt_ = {1, 0, 0, 0};
        cp_vect_ = {0, 0, -(nose_to_cp_ - nose_to_cg_)};
        rasaero_import_ = rasaero;
    }

    /*************************** Get parameters *****************************#*/
    Vector3d get_r_vect() const { return r_vect_; };
    Vector3d get_r_dot() const { return r_dot_; };
    Vector3d get_r_ddot() const { return r_ddot_; };

    Quaterniond get_q_ornt() const { return q_ornt_; };

    Vector3d get_w_vect() const { return w_vect_; };
    Vector3d get_w_dot() const { return w_dot_; };

    Vector3d get_f_net() const { return f_net_; };
    Vector3d get_m_net() const { return m_net_; };

    double get_structural_mass() const { return structural_mass_; };
    double get_total_mass() const { return total_mass_; };
    double get_mach() const { return mach_; };
    double get_alpha() const { return alpha_; };
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

    Vector3d get_launch_ecef() const { return launch_ecef_; };
    Vector3d get_launch_geod() const { return launch_geod_; };

    std::array<double, 9> get_I() const { return I_; };

    std::shared_ptr<Flaps> get_flaps() { return flaps_; }

    /**************************** Set parameters ******************************/
    void set_r_vect(Vector3d vector) { r_vect_ = vector; };
    void set_r_dot(Vector3d vector) { r_dot_ = vector; };
    void set_r_ddot(Vector3d vector) { r_ddot_ = vector; };

    void set_q_ornt(Quaterniond quatrn) { q_ornt_ = quatrn; };

    void set_I(const std::array<double, 9>& array) { I_ = array; };

    void set_w_vect(Vector3d vector) { w_vect_ = vector; };
    void set_w_dot(Vector3d vector) { w_dot_ = vector; };

    void set_f_net(Vector3d vector) { f_net_ = vector; };
    void set_m_net(Vector3d vector) { m_net_ = vector; };

    void set_structural_mass(double mass) { structural_mass_ = mass; };
    void set_total_mass(double mass) { total_mass_ = mass; };
    void set_mach(double mach) { mach_ = mach; };
    void set_alpha(double alpha) { alpha_ = alpha; };
    void set_reference_length(double d_ref) { reference_length_ = d_ref; };
    void set_reference_area(double a_ref) { reference_area_ = a_ref; };
    void set_total_normal_force_coeff(double cn_total) {
        total_normal_force_coeff_ = cn_total;
    };
    void set_total_axial_force_coeff(double ca_total) {
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

    void set_flaps(std::shared_ptr<Flaps> flaps) { flaps_ = flaps; };

    /************************ Internal State Update ***************************/
    void update_aero_coefficients(bool poweron);
    void update_flaps(double tStep) { flaps_->update(tStep); };

    /********************** Reference Frame Conversions ***********************/
    // Converts arbitrary vector to/from ENU frame and rocket reference frame
    Vector3d enu2r(Vector3d vector);
    Vector3d r2enu(Vector3d vector);

    // Converts rocket position vector to/from ENU frame and ECEF frame
    Vector3d position_enu2ecef(Vector3d pos_enu);
    Vector3d position_ecef2enu(Vector3d pos_ecef);

    // Converts arbitrary vector to/from ENU frame and ECEF frame
    Vector3d enu2ecef(Vector3d vector);
    Vector3d ecef2enu(Vector3d vector);

    // Converts rocket position  vector from ECEF frame to Geodetic coordinates
    Vector3d ecef2geod(Vector3d ecef);

    // Returns the *unit* vector in the direction of gravity in various frames
    Vector3d gravity_vector_ecef();
    Vector3d gravity_vector_enu();
    Vector3d gravity_vector_rf();

   private:
    // The following are in ENU frame
    Vector3d r_vect_{0, 0, 0};  // r vector
    Vector3d r_dot_{0, 0, 0};   // r-dot (velocity)
    Vector3d r_ddot_{0, 0, 0};  // r-double-dot (acceleration)
    Vector3d w_vect_{0, 0, 0};  // angular velocity (omega) vector
    Vector3d w_dot_{0, 0, 0};   // angular acceleration vector

    // The following are in ENU frame
    Vector3d f_net_{0, 0, 0};  // net force in Netwons
    Vector3d m_net_{0, 0, 0};  // net moment in Newton*meters

    Quaterniond q_ornt_{};  // ENU -> rocket frame quaternion

    Vector3d cp_vect_{};  // CG to CP vector, rocket frame

    //---------- Launch Pad Location ----------
    // The following are in Geocentric frame
    // Vector3d launch_ecef_{150992.99, -4882549.85, 4087626.55};
    // Vector3d launch_geod_{40.111801, -88.228691, 216};
    // (40.111801, -88.228691, 216) - Talbot Laboratory
    // Spaceport America
    Vector3d launch_ecef_{-1563305, -5123022, 3453817};
    Vector3d launch_geod_{32.990278, -106.969722, 1401};

    //---------- Intertial Parameters ----------
    std::array<double, 9> I_{};       // Rocket moment of inertia tensor
    double structural_mass_ = 25.91;  // structural mass of rocket in Kg
    double total_mass_ = 25.91;       // total mass including propellant in Kg
    double nose_to_cg_ = 1.683;       // nosecone tip to CG distance in m

    //----------- Aerodynamic Parameters ----------
    std::shared_ptr<RASAeroImport> rasaero_import_;  // Aero lookup table
    double reference_length_ = 3.2258;               // reference length in m
    double reference_area_ = 0.00811;                // reference area in m^2
    double total_normal_force_coeff_ = 9.65;  // total normal force coefficient
    double total_axial_force_coeff_ = 0.630;  // total axial force coefficient
    double nose_to_cp_ = 4.03;  // nosecone tip to Cp distance in m
    double mach_ = 0.0;         // Freestream air mach number
    double alpha_ = 0.0;        // Rocket total angle-of-attack to air

    //----------- Control Surfaces ---------
    std::shared_ptr<Flaps> flaps_;
};

#endif
