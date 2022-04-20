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
#include <cmath>
#include <iostream>

using Eigen::Quaterniond;
using Eigen::Vector3d;


/*****************************************************************************/
/*                     INTERNAL STATE UPDATE FUNCTIONS                       */
/*****************************************************************************/

/**
 * @brief Updates the internally stored aerodynamic coefficients of the Rocket
 * obtained from the RASAero lookup table
 *
 * Function fails gracefully without mutating anything if a RASAeroImport class
 * is not associated with this rocket.
 *
 * @param poweron True if the rocket motor is currently burning
 * @param protuberance The current amount of protuberance [0.0 - 1.0]
 */
void Rocket::update_aero_coefficients(bool poweron, double protuberance) {
    constexpr double kInchToMeters = 0.0254;
    if (rasaero_import_) {
        RASAeroCoefficients coefficients =
            rasaero_import_->get_aero_coefficients(mach_, alpha_, protuberance);

        if (poweron) {
            set_total_axial_force_coeff(coefficients.ca_poweron);
        } else {
            set_total_axial_force_coeff(coefficients.ca_poweroff);
        }

        set_total_normal_force_coeff(coefficients.cn_total);
        set_nose_to_cp(coefficients.cp_total * kInchToMeters);
    }
}

/*****************************************************************************/
/*                       ENU FRAME <-> ROCKET FRAME                          */
/*****************************************************************************/

/**
 * @brief Performs a quaternion rotation to translate a vector from the inertial
 * frame to the rocket body frame.
 *
 * @param vector Input vector in intertial frame to be rotated
 * @return Vector3d The rotated vector represented in the rocket body frame
 */
Vector3d Rocket::enu2r(Vector3d vector) {
    Quaterniond p{0, vector.x(), vector.y(), vector.z()};
    p = (q_ornt_.conjugate() * p) * q_ornt_;
    return p.vec();
}

/**
 * @brief Performs a quaternion rotation to translate a vector from the rocket
 * frame to the ENU reference frame.
 *
 * @param vector Input vector in rocket frame to be rotated
 * @return Vector3d The rotated vector represented in the ENU frame
 */
Vector3d Rocket::r2enu(Vector3d vector) {
    Quaterniond p{0, vector.x(), vector.y(), vector.z()};
    p = (q_ornt_ * p) * q_ornt_.conjugate();

    return p.vec();
}

/*****************************************************************************/
/*                        ENU FRAME <-> ECEF FRAME                           */
/*****************************************************************************/
/*
 * ENU is the distance East, North, and Up the rocket is from the origin in
 * meters with East-North plane tangent to the earth, and an arbitrary but fixed
 * origin. In the context of SILSIM, the ENU frame has its origin at the launch
 * pad.
 *
 * ECEF is the distance the rocket is from the center of mass of the earth:
 * The X axis points to the prime meridian at 0deg longitude, on the equator
 * The Y axis points 90deg east and lies on the equator
 * The Z axis points north to form a right-handed coordinate system
 *
 * https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
 * "From ENU to ECEF"
 */

/**
 * @brief Converts launchpad-relative ENU coordinates to ECEF coordinates
 *
 * Method takes in the position of the rocket relative to the launchpad in the
 * East-North-Up reference frame and pulls the latitude and longitude of the
 * launchpad from the rocket class.  The launchpad coordinates are converted
 * into radians and used in the transformation matrix.
 *
 * The transformation matrix and the Earth-Centered-Earth-Fixed (ecef)
 * coordinates of the launchpad are then used to calculate the ecef position of
 * the rocket.
 *
 * @param pos_enu Position of the rocket in the East-North-Up coordinate system
 * @return Vector3d Position of the rocket in the Earth-Centered-Earth-Fixed
 * coordinte system
 */
Vector3d Rocket::position_enu2ecef(Vector3d pos_enu) {
    using std::sin, std::cos;

    // longitude and latitude of launchpad converted to radians
    double lambda = get_launch_geod().y() * M_PI / 180;
    double lat = get_launch_geod().x() * M_PI / 180;

    Eigen::Matrix3d transform{
        {-sin(lambda), -sin(lat) * cos(lambda), cos(lat) * cos(lambda)},
        {cos(lambda), -sin(lat) * sin(lambda), cos(lat) * sin(lambda)},
        {0, cos(lat), sin(lat)}};

    return (transform * pos_enu) + get_launch_ecef();
}

/**
 * @brief Converts ECEF coordinates to launchpad-relative ENU
 * coordinates
 *
 * Method takes in the position of the rocket in the Earth-Centered-Earth-Fixed
 * reference frame and subtracts the ECEF coordinates of the launch pad to yield
 * a vector pointing from the launchpad to the rocket.
 *
 * It then applies the inverse of the rotation matrix used to go from ENU to
 * ECEF to perform the opposite rotation.
 *
 * @param pos_ecef Position of the rocket in the Earth-Centered-Earth-Fixed
 * coordinate system
 * @return Vector3d Position of the rocket in the East-North-Up coordinate
 * system
 */
Vector3d Rocket::position_ecef2enu(Vector3d pos_ecef) {
    using std::sin, std::cos;

    // longitude and latitude of launchpad converted to radians
    double lambda = get_launch_geod().y() * M_PI / 180;
    double lat = get_launch_geod().x() * M_PI / 180;

    Eigen::Matrix3d transform{
        {-sin(lambda), -sin(lat) * cos(lambda), cos(lat) * cos(lambda)},
        {cos(lambda), -sin(lat) * sin(lambda), cos(lat) * sin(lambda)},
        {0, cos(lat), sin(lat)}};

    return transform.transpose() * (pos_ecef - get_launch_ecef());
}

/**
 * @brief Transforms a vector from the ENU frame to the ECEF reference frame
 *
 * Method applies a rotation matrix to transform a vector from the East-North-Up
 * reference frame to the Earth-Centered-Earth-Fixed reference frame. The
 * rotation matrix is constructed using the launchpad location stored in the
 * Rocket class.
 *
 * @param vector A vector expressed in the East-North-Up reference frame
 * @return Vector3d The same vector, now expressed in the
 * Earth-Centered-Earth-Fixed reference frame
 */
Vector3d Rocket::enu2ecef(Vector3d vector) {
    using std::sin, std::cos;

    // longitude and latitude of launchpad converted to radians
    double lambda = get_launch_geod().y() * M_PI / 180;
    double lat = get_launch_geod().x() * M_PI / 180;

    Eigen::Matrix3d transform{
        {-sin(lambda), -sin(lat) * cos(lambda), cos(lat) * cos(lambda)},
        {cos(lambda), -sin(lat) * sin(lambda), cos(lat) * sin(lambda)},
        {0, cos(lat), sin(lat)}};

    return transform * vector;
}

/**
 * @brief Transforms a vector from the ECEF frame to the ENU reference frame
 *
 * Method applies a rotation matrix to transform a vector from the
 * Earth-Centered-Earth-Fixed reference frame to the East-North-Up reference
 * frame. The rotation matrix is constructed using the launchpad location stored
 * in the Rocket class.
 *
 * @param vector A vector expressed in the Earth-Centered-Earth-Fixed reference
 * frame
 * @return Vector3d The same vector, now expressed in the East-North-Up
 * reference frame
 */
Vector3d Rocket::ecef2enu(Vector3d vector) {
    using std::sin, std::cos;

    // longitude and latitude of launchpad converted to radians
    double lambda = get_launch_geod().y() * M_PI / 180;
    double lat = get_launch_geod().x() * M_PI / 180;

    Eigen::Matrix3d transform{
        {-sin(lambda), -sin(lat) * cos(lambda), cos(lat) * cos(lambda)},
        {cos(lambda), -sin(lat) * sin(lambda), cos(lat) * sin(lambda)},
        {0, cos(lat), sin(lat)}};

    return transform.transpose() * vector;
}

/*****************************************************************************/
/*                      ECEF FRAME <-> GEODETIC FRAME                        */
/*****************************************************************************/

/**
 * @brief Converts the rocket's Earth-Centered-Earth-Fixed coordinates into
 * Geodetic latitude, longitude, and altitude
 *
 * Method takes in the Earth-Centered-Earth-Fixed coordinates of the rocket.  It
 * then performs a series of calculations to determine the Geodetic latitude,
 * longitude, and altitude of the rocket.  Variable names other than "geod" are
 * not significant and only come from the reference math.
 *
 * ECEF is the distance the rocket is from the center of mass of the earth
 * The X axis points to the prime meridian at 180deg longitude, on the equator
 * The Y axis points 90deg east and lies on the equator
 * The Z axis points north to form a right-handed coordinate system
 *
 * Geodetic is the latitude, longitude, and altitude of the rocket based on a
 * spheroid (squished) earth
 *
 * https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
 * "The application of Ferrari's solution"
 *
 * @param ecef Earth-Centered-Earth-Fixed position of the rocket
 * @return Vector3d Geodetic latitude, longitude, and altitude of the rocket
 */
Vector3d Rocket::ecef2geod(Vector3d ecef) {
    Vector3d geod;

    const double a = 6378137.0;
    const double b = 6356752.3142;
    const double e2 = (std::pow(a, 2) - std::pow(b, 2)) / std::pow(a, 2);
    const double er2 = (std::pow(a, 2) - std::pow(b, 2)) / std::pow(b, 2);
    double p = std::sqrt(std::pow(ecef.x(), 2) + std::pow(ecef.y(), 2));
    double F = 54 * std::pow(b, 2) * std::pow(ecef.z(), 2);
    double G = std::pow(p, 2) + ((1 - e2) * std::pow(ecef.z(), 2)) -
               (e2 * (std::pow(a, 2) - std::pow(b, 2)));
    double c = (std::pow(e2, 2) * F * std::pow(p, 2)) / std::pow(G, 3);
    double s = std::cbrt(1 + c + std::sqrt(std::pow(c, 2) + (2 * c)));
    double k = s + 1 + (1 / s);
    double P = F / (3 * std::pow(k, 2) * std::pow(G, 2));
    double Q = std::sqrt(1 + (2 * std::pow(e2, 2) * P));
    double r0 =
        ((-P * e2 * p) / (1 + Q)) +
        std::sqrt(((0.5 * std::pow(a, 2)) * (1 + (1 / Q))) -
                  ((P * (1 - e2) * std::pow(ecef.z(), 2)) / (Q * (1 + Q))) -
                  (0.5 * P * std::pow(p, 2)));
    double U = std::sqrt(std::pow((p - (e2 * r0)), 2) + std::pow(ecef.z(), 2));
    double V = std::sqrt(std::pow(p - (e2 * r0), 2) +
                         ((1 - e2) * std::pow(ecef.z(), 2)));
    double z0 = (std::pow(b, 2) * ecef.z()) / (a * V);

    geod.z() = U * (1 - (std::pow(b, 2) / (a * V)));
    geod.x() = std::atan((ecef.z() + (er2 * z0)) / p) * 180 / M_PI;
    geod.y() = std::atan2(ecef.y(), ecef.x()) * 180 / M_PI;

    return geod;
}

/*****************************************************************************/
/*                         GRAVITY DIRECTION VECTOR                          */
/*****************************************************************************/
/*
 * The following functions return _unit_ vectors in the direction of the gravity
 * vector acting on the rocket's center of mass.
 *
 * This gravity vector is easiest to achieve using the ECEF position vector of
 * the rocket. Simply normalizing the ECEF position vector and inverting it
 * yields the gravity direction vector.
 */

/**
 * @brief Returns a unit gravity direction vector in the ECEF reference frame
 *
 * @return Vector3d Gravity direction vector in ECEF frame
 */
Vector3d Rocket::gravity_vector_ecef() {
    Vector3d pos_ecef = position_enu2ecef(r_vect_);
    return -pos_ecef.normalized();
}

/**
 * @brief Returns a unit gravity direction vector in the ENU reference frame
 *
 * @return Vector3d Gravity direction vector in ENU frame
 */
Vector3d Rocket::gravity_vector_enu() {
    Vector3d grav_ecef = gravity_vector_ecef();
    return ecef2enu(grav_ecef);
}

/**
 * @brief Returns a unit gravity direction vector in the Rocket body reference
 * frame
 *
 * @return Vector3d Gravity direction vector in Rocket body frame
 */
Vector3d Rocket::gravity_vector_rf() {
    Vector3d grav_enu = gravity_vector_enu();
    return enu2r(grav_enu);
}
