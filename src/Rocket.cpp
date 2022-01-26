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
