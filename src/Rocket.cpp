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



#include <Eigen/Dense>

using Eigen::Vector3d;
using Eigen::Quaterniond;

Rocket::Rocket() {
    q_ornt_ = Quaterniond(1, 0, 0, 0);

    Cp_vect_ = Vector3d(0, 0, -(nose_to_cp_ - nose_to_cg_));

    r_vect_ = Vector3d(0, 0, 0);
    r_dot_ = Vector3d(0, 0, 0);
    r_ddot_ = Vector3d(0, 0, 0);
    w_vect_ = Vector3d(0, 0, 0);
    w_dot_ = Vector3d(0, 0, 0);
    f_net_ = Vector3d(0, 0, 0);
    t_net_ = Vector3d(0, 0, 0);
    


}

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
    Cp_vect_.x() = 0;
    Cp_vect_.y() = 0;
    Cp_vect_.z() = -(nose_to_cp_ - nose_to_cg_);
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
    Cp_vect_.x() = 0;
    Cp_vect_.y() = 0;
    Cp_vect_.z() = -(nose_to_cp_ - nose_to_cg_);
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
Vector3d Rocket::i2r(Vector3d vector) {
    Quaterniond p(0, vector.x(), vector.y(), vector.z());
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
Vector3d Rocket::r2i(Vector3d vector) {
    Quaterniond p(0, vector.x(), vector.y(), vector.z());
    p = (q_ornt_ * p) * q_ornt_.conjugate();
    
    return p.vec();
}
