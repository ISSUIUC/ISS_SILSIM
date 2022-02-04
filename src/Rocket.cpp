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

using Eigen::Quaterniond;
using Eigen::Vector3d;

/**
 * @brief Performs a quaternion rotation to translate a vector from the inertial
 * frame to the rocket body frame.
 *
 * @param vector Input vector in intertial frame to be rotated
 * @return Vector3d The rotated vector represented in the rocket body frame
 */
Vector3d Rocket::i2r(Vector3d vector) {
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
Vector3d Rocket::r2i(Vector3d vector) {
    Quaterniond p{0, vector.x(), vector.y(), vector.z()};
    p = (q_ornt_ * p) * q_ornt_.conjugate();

    return p.vec();
}

void Rocket::update_aero_coefficinets(bool poweron,
                                      double protuberance_percent) {
    RASAeroCoefficients coefficients = rasaero_import_->get_aero_coefficients(
        mach_, alpha_, protuberance_percent);

    if (poweron) {
        set_total_axial_focre_coeff(coefficients.ca_poweron);
    } else {
        set_total_axial_focre_coeff(coefficients.ca_poweroff);
    }

    set_total_normal_force_coeff(coefficients.cn_total);
    set_nose_to_cp(coefficients.cp_total);
}
