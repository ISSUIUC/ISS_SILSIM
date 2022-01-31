/**
 * @file        Rocket.cpp
 * @authors     Ayberk Yaraneri
 *
 * @brief       Rocket class member function implementations
 *
 * The rocket class encapsulates all physical quantities and parameters of the
 * Rocket. The class also contains references to child objects representing
 * components like sensors and rocket motors along with mechanisms to extract
 * useful information about the rocket's state/trajectory.
 *
 */

#include "Rocket.h"

#include <Atmosphere.h>

#include <Eigen/Dense>

using Eigen::Quaterniond;
using Eigen::Vector3d;

/**
 * @brief Obtain distance between tip of nose and the center of gravity
 *
 * @param nose_to_cg Reference to double to overwrite with calculated distance
 */
void Rocket::get_nose_to_cg(double& nose_to_cg) const {
    nose_to_cg = nose_to_cg_;
}

/**
 * @brief Set the distance between tip of nose and the center of gravity
 *
 * Currently assumes the CG is perfectly aligned with the axis of the rocket.
 * i.e. the x and y position of the CG is zero in the body frame
 *
 * @param nose_to_cg Distance between tip of nosecone and CG
 */
void Rocket::set_nose_to_cg(double& nose_to_cg) {
    nose_to_cg_ = nose_to_cg;
    Cp_vect_ = {0, 0, -(nose_to_cp_ - nose_to_cg_)};
}

/**
 * @brief Obtain distance between tip of nose and the center of pressure
 *
 * @param nose_to_cp Reference to double to overwrite with calculated distance
 */
void Rocket::get_nose_to_cp(double& nose_to_cp) const {
    nose_to_cp = nose_to_cp_;
}

/**
 * @brief Set the distance between tip of nose and the center of pressure
 *
 * Currently assumes the CP is perfectly aligned with the axis of the rocket.
 * i.e. the x and y position of the CP is zero in the body frame
 *
 * @param nose_to_cg Distance between tip of nosecone and CG
 */
void Rocket::set_nose_to_cp(double& nose_to_cp) {
    nose_to_cp_ = nose_to_cp;
    Cp_vect_ = {0, 0, -(nose_to_cp_ - nose_to_cg_)};
}

/**
 * @brief Obtain vector pointing from CG -> CP
 *
 * @param vector Reference to a vector to overwrite with CG-to-CP vector
 */
void Rocket::get_Cp_vect(Vector3d& vector) const { vector = Cp_vect_; }

/**
 * @brief Performs a quaternion rotation to translate a vector from the inertial
 * frame to the rocket body frame.
 *
 * @param vector Input vector in intertial frame to be rotated
 * @return Vector3d The rotated vector represented in the rocket body frame
 */
Vector3d Rocket::i2r(Vector3d vector) const {
    Quaterniond p{0, vector.x(), vector.y(), vector.z()};
    p = (q_ornt_.conjugate() * p) * q_ornt_;
    return p.vec();
}

/**
 * @brief Performs a quaternion rotation to translate a vector from the rocket
 * frame to the inertial reference frame.
 *
 * @param vector Input vector in rocket frame to be rotated
 * @return Vector3d The rotated vector represented in the inertial frame
 */
Vector3d Rocket::r2i(Vector3d vector) const {
    Quaterniond p{0, vector.x(), vector.y(), vector.z()};
    p = (q_ornt_ * p) * q_ornt_.conjugate();

    return p.vec();
}

void Rocket::update_parachutes() {
    main_chute.update(get_r_vect(), get_r_dot());
    drogue_chute.update(get_r_vect(), get_r_dot());
}
Vector3d Rocket::calculate_drag_rf() const {
    double atmosphere_density = Atmosphere::get_density(get_r_dot().z());
    auto vel_rf = i2r(get_r_dot());

    if(main_chute.is_open()){
        double drag_mag = 0.5 * main_chute.area() * main_chute.drag_coefficient() * vel_rf.squaredNorm() * atmosphere_density;
        return -drag_mag * vel_rf.normalized();
    } else if(drogue_chute.is_open()){
        double drag_mag = 0.5 * drogue_chute.area() * drogue_chute.drag_coefficient() * vel_rf.squaredNorm() * atmosphere_density;
        return -drag_mag * vel_rf.normalized();
    } else {
        // angle between velocity
        double alpha;
        if(vel_rf.norm() == 0){
            alpha = 0;
        } else {
            alpha = acos(vel_rf.z() / vel_rf.norm());
        }
        // vector and rocket axis
        double normal_coef = get_Cna() * alpha;

        double normal_force_mag = 0.5 * normal_coef * vel_rf.squaredNorm() *
                                  atmosphere_density * get_A_ref();
        Eigen::Vector3d normal_force_rf = {(-vel_rf.x()), (-vel_rf.y()), 0};

        normal_force_rf.normalize();
        normal_force_rf = normal_force_rf * normal_force_mag;

        double drag_mag = 0.5 * get_Cd() * vel_rf.squaredNorm() * get_A_ref() * atmosphere_density;
        Vector3d drag_rf{0, 0, std::copysign(drag_mag, -vel_rf.z())};

        return normal_force_rf + drag_rf;
    }
}
