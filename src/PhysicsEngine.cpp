/**
 * @file        PhysicsEngine.cpp
 * @authors     Ayberk Yaraneri
 *              Jacob Gugala
 *
 * @brief       PhysicsEngine class member function implementations
 *
 * The PhysicsEngine class encapsulates a particular algorithm that is used
 * to advance the state of the simulation in time. These algorithms are
 * commonly responsible for calculating time varying quantities that govern
 * the trajectory of the rocket, such as angular and axial acceleration.
 *
 */

#include "PhysicsEngine.h"

#include <Eigen/Dense>
#include <cmath>

#include "Atmosphere.h"

using Eigen::Quaterniond;
using Eigen::Vector3d;

#define RAD2DEG (180.0 / 3.14159265)

ForwardEuler::ForwardEuler(Rocket& rocket, SolidMotor& motor)
    : PhysicsEngine(rocket, motor) {
    euler_logger =
        spdlog::basic_logger_mt("Euler_Logger", "logs/forward_euler.log");
}

/**
 * @brief Calculates forces and moments and integrates with a simple euler step
 *
 * @param tStamp Current simulation timestamp
 * @param tStep Simulation time step size
 */
void ForwardEuler::march_step(double tStamp, double tStep) {
    // {variable}_rf = rocket frame (stuck to rocket)
    // {variable}_enu = ENU frame (stuck to earth)
    // f_{variable} = force
    // t_{variable} = torque

    /*************** Retrieve instantaneous rocket parameters *****************/

    // ENU frame dynamics parameters
    Vector3d r_vect_enu = rocket_.get_r_vect();  // position
    Vector3d r_dot_enu = rocket_.get_r_dot();    // velocity
    Vector3d r_ddot_enu = rocket_.get_r_ddot();  // acceleration
    Vector3d w_vect_enu = rocket_.get_w_vect();  // angular velocity (omega)
    Vector3d w_dot_enu = rocket_.get_w_dot();    // angular acceleration
    Vector3d f_net_enu = rocket_.get_f_net();    // net force (Newtons)
    Vector3d t_net_enu = rocket_.get_t_net();    // net torque (Newtons*meters)

    // Quaternion from ENU to rocket frame
    Quaterniond q_ornt = rocket_.get_q_ornt();  // orientation of rocket

    // CG to Cp vector
    Vector3d Cp_vect_rf =
        rocket_.get_Cp_vect();  // CG to Cp (center of pressure) vector

    // Get moment of inertia tenspr
    double I_tens[9];
    rocket_.get_I(I_tens);  // moment of inertia

    // parameters
    double mass = rocket_.get_mass();    // mass of rocket
    double A_ref = rocket_.get_A_ref();  // ref area in m^2
    double c_Na = rocket_.get_Cna();     // normal force coefficient derivative
    double c_D = rocket_.get_Cd();       // drag coefficient

    // Motor thrust vector, rocket frame
    Vector3d thrust_rf =
        motor_.get_thrust(tStamp);  // thrust of rocket at current timestamp

    /********************* Calculate forces and torques ***********************/
    Vector3d f_aero_rf;   // Aerodynamic forces, rocket frame
    Vector3d t_aero_rf;   // Aerodynamic torques, rocket frame
    Vector3d f_aero_enu;  // Aerodynamic forces, ENU frame
    Vector3d t_aero_enu;  // Aerodynamic torques, ENU frame

    Vector3d f_net_rf;  // net force
    Vector3d t_net_rf;  // net torque

    // Set aerodynamic forces and torques to zero if velocity is small. This
    // avoids calculations returning NaN values.
    if (r_dot_enu.norm() > 0.01) {
        Vector3d rocket_axis_rf(0, 0, 1);
        Vector3d v_rf = rocket_.enu2r(r_dot_enu);
        double alpha =
            acos(v_rf.z() /
                 v_rf.norm());  // angle between velocity vector and rocket axis
        Vector3d f_N_rf;        // normal aerodynamic force

        double c_N = c_Na * alpha;
        double f_N_mag = c_N * 0.5 * Atmosphere::get_density(r_vect_enu.z()) *
                         v_rf.squaredNorm() *
                         A_ref;  // norm of normal force (assuming constant
                                 // 0.5 is a coefficient in the equation

        f_N_rf.x() = (-v_rf.x());
        f_N_rf.y() = (-v_rf.y());
        f_N_rf.z() = 0;

        f_N_rf.normalize();
        f_N_rf = f_N_rf * f_N_mag;

        double f_D_mag = c_D * 0.5 * Atmosphere::get_density(r_vect_enu.z()) *
                         v_rf.squaredNorm() * A_ref;
        // make drag force apply in the opposite direction to rocket travel
        Vector3d f_D_rf(0, 0, std::copysign(f_D_mag, -v_rf.z()));

        f_aero_rf = f_N_rf + f_D_rf;
        t_aero_rf = Cp_vect_rf.cross(f_aero_rf);

    } else {
        f_aero_rf.x() = 0;
        f_aero_rf.y() = 0;
        f_aero_rf.z() = 0;

        t_aero_rf.x() = 0;
        t_aero_rf.y() = 0;
        t_aero_rf.z() = 0;
    }

    f_net_enu = rocket_.r2enu(f_aero_rf + thrust_rf);
    f_net_enu.z() -= (9.81 * mass);

    t_net_rf = t_aero_rf;
    t_net_enu = rocket_.r2enu(t_aero_rf);

    /************************** Perform euler step ****************************/

    r_vect_enu += r_dot_enu * tStep;
    r_dot_enu += r_ddot_enu * tStep;
    r_ddot_enu = f_net_enu / mass;

    q_ornt = update_quaternion(q_ornt, w_vect_enu, tStep);

    w_vect_enu += w_dot_enu * tStep;

    w_dot_enu.x() = t_net_enu.x() / I_tens[0];
    w_dot_enu.y() = t_net_enu.y() / I_tens[4];
    w_dot_enu.z() = t_net_enu.z() / I_tens[8];

    // Naively accounting for launch rail
    if (r_vect_enu.norm() < 4.50) {
        w_dot_enu.x() = 0;
        w_dot_enu.y() = 0;
        w_dot_enu.z() = 0;
        w_vect_enu.x() = 0;
        w_vect_enu.y() = 0;
        w_vect_enu.z() = 0;
    }

    euler_logger->debug("Timestamp {}", tStamp);
    euler_logger->debug("thrust_rf = <{}, {}, {}>", thrust_rf.x(),
                        thrust_rf.y(), thrust_rf.z());
    euler_logger->debug("f_aero_rf = <{}, {}, {}>", f_aero_rf.x(),
                        f_aero_rf.y(), f_aero_rf.z());
    euler_logger->debug("t_aero_rf = <{}, {}, {}>", t_aero_rf.x(),
                        t_aero_rf.y(), t_aero_rf.z());
    euler_logger->debug("t_net_rf = <{}, {}, {}>", t_net_rf.x(), t_net_rf.y(),
                        t_net_rf.z());
    euler_logger->debug("f_net_enu = <{}, {}, {}>", f_net_enu.x(),
                        f_net_enu.y(), f_net_enu.z());
    euler_logger->debug("r_dot_enu = <{}, {}, {}>", r_dot_enu.x(),
                        r_dot_enu.y(), r_dot_enu.z());
    euler_logger->debug("r_ddot_enu = <{}, {}, {}>", r_ddot_enu.x(),
                        r_ddot_enu.y(), r_ddot_enu.z());

    rocket_.set_r_vect(r_vect_enu);
    rocket_.set_r_dot(r_dot_enu);
    rocket_.set_r_ddot(r_ddot_enu);
    rocket_.set_w_vect(w_vect_enu);
    rocket_.set_w_dot(w_dot_enu);
    rocket_.set_f_net(f_net_enu);
    rocket_.set_t_net(t_net_enu);
    rocket_.set_q_ornt(q_ornt);
}

/**
 * @brief Performs a fourth-order Runge Kutta method to advance the physics
 * forward one timestep
 *
 * Method calculates four different rocket states over the course of the time
 * step, then takes a weighted average to more accuratly simulate the state of
 * the rocket after the full time step. The first state is the current state of
 * the rocket.  The second state is a basic Euler step from the first state
 * using half of the time step.  The third state is an Euler step from the first
 * state using the forces, velocities, and accelerations of the second state and
 * half of the time step.  The fourth state is an Euler step from the first
 * state using the data from the third state and the full time step.
 *
 * Equations (page 4):
 * https://github.com/ISSUIUC/ISS_SILSIM/blob/master/docs/MIT18_330S12_Chapter5.pdf
 * Visual representation:
 * https://www.haroldserrano.com/blog/visualizing-the-runge-kutta-method
 *
 * @param tStamp Specific time stamp in the simulation
 * @param tStep Simulation time step size
 */
void RungeKutta::march_step(double tStamp, double tStep) {
    /*************** Retrieve Instantaneous Rocket Parameters *****************/

    Vector3d pos_enu = rocket_.get_r_vect();
    Vector3d vel_enu = rocket_.get_r_dot();
    Vector3d accel_enu = rocket_.get_r_ddot();
    Vector3d ang_vel_enu = rocket_.get_w_vect();
    Vector3d ang_accel_enu = rocket_.get_w_dot();

    Quaterniond orient = rocket_.get_q_ornt();

    double inertia[9];  // moments of inertia
    rocket_.get_I(inertia);

    double mass = rocket_.get_mass();

    Vector3d geod = rocket_.get_r_geod();
    Vector3d ecef = rocket_.get_r_ecef();

    /******************** Calculate Intermediate States **********************/
    // Each state is used to calculate the next state

    // Reformating of the initial state of the rocket
    RungeKuttaState k1{pos_enu, vel_enu, accel_enu, ang_vel_enu, ang_accel_enu};
    // Calculated with Euler step using half tStep and initial state
    RungeKuttaState k2 = calc_state(tStamp, 0.5 * tStep, k1);
    // Calculated with Euler step using half tStep and k2 state
    RungeKuttaState k3 = calc_state(tStamp + (0.5 * tStep), 0.5 * tStep, k2);
    // Calculated with Euler step using full tStep and k3 state
    RungeKuttaState k4 = calc_state(tStamp + (0.5 * tStep), tStep, k3);

    /********************** Perform Runge-Kutta Method ************************/

    // calculate weighted averages
    Vector3d vel_avg = (k1.vel + (2 * k2.vel) + (2 * k3.vel) + k4.vel) / 6;
    Vector3d accel_avg =
        (k1.accel + (2 * k2.accel) + (2 * k3.accel) + k4.accel) / 6;
    Vector3d ang_vel_avg =
        (k1.ang_vel + (2 * k2.ang_vel) + (2 * k3.ang_vel) + k4.ang_vel) / 6;
    Vector3d ang_accel_avg = (k1.ang_accel + (2 * k2.ang_accel) +
                              (2 * k3.ang_accel) + k4.ang_accel) /
                             6;

    // calculate rocket data based on average values instaed of initial
    pos_enu += tStep * vel_avg;
    vel_enu += tStep * accel_avg;

    Vector3d net_force_enu = calc_net_force(tStamp, pos_enu, vel_avg);
    Vector3d net_torque_enu = calc_net_torque(vel_avg, pos_enu);

    accel_enu = net_force_enu / mass;

    ang_vel_enu += tStep * ang_accel_avg;
    ang_accel_enu.x() = net_torque_enu.x() / inertia[0];
    ang_accel_enu.y() = net_torque_enu.y() / inertia[4];
    ang_accel_enu.z() = net_torque_enu.z() / inertia[8];

    orient = update_quaternion(orient, ang_vel_avg, tStep);

    //---- Launch Rail ----
    // very basic implementation
    if (pos_enu.norm() < 4.50) {
        ang_vel_enu.x() = 0;
        ang_vel_enu.y() = 0;
        ang_vel_enu.z() = 0;
        ang_accel_enu.x() = 0;
        ang_accel_enu.y() = 0;
        ang_accel_enu.z() = 0;
    }

    //---- Set Values ----
    rocket_.set_r_vect(pos_enu);
    rocket_.set_r_dot(vel_enu);
    rocket_.set_r_ddot(accel_enu);
    rocket_.set_w_vect(ang_vel_enu);
    rocket_.set_w_dot(ang_accel_enu);
    rocket_.set_f_net(net_force_enu);
    rocket_.set_t_net(net_torque_enu);
    rocket_.set_q_ornt(orient);

    rocket_.set_r_ecef(i2ecef(pos_enu));
    rocket_.set_r_geod(ecef2geod(ecef));
}

/**
 * @brief Takes in time and velocity to calculate net force
 *
 * @param tStamp Specific time stamp in the simulation
 * @param vel_enu Rocket's velocity with respect to the ENU frame
 * @return Vector3  Vector containing the net force on the rocket in the x, y,
 * and z directions
 */
Vector3d RungeKutta::calc_net_force(double tStamp, Vector3d pos_enu,
                                    Vector3d vel_enu) {
    // {variable}_rf = rocket frame (stuck to rocket)
    // {variable}_enu = ENU frame (stuck to earth)

    /*************** Retrieve Instantaneous Rocket Parameters *****************/

    Vector3d thrust_rf = motor_.get_thrust(tStamp);

    double mass = rocket_.get_mass();
    double area = rocket_.get_A_ref();
    double c_Na = rocket_.get_Cna();  // normal force coefficient derivative
    double drag_coef = rocket_.get_Cd();

    Vector3d geod = ecef2geod(i2ecef(pos_enu));

    /************************* Calculate Net Force ****************************/

    Vector3d aero_force_rf;

    if (vel_enu.norm() > 0.01) {
        // i2r pulls a quaternion from the rocket, be sure to set orientation
        // beforehand
        Vector3d vel_rf = rocket_.enu2r(vel_enu);
        Vector3d normal_force_rf;

        double alpha =
            acos(vel_rf.z() / vel_rf.norm());  // angle between velocity
                                               // vector and rocket axis
        double normal_coef = c_Na * alpha;

        double normal_force_mag = 0.5 * normal_coef * vel_rf.squaredNorm() *
                                  area * Atmosphere::get_density(geod.z());
        normal_force_rf = {(-vel_rf.x()), (-vel_rf.y()), 0};

        normal_force_rf.normalize();
        normal_force_rf = normal_force_rf * normal_force_mag;

        double drag_mag = 0.5 * drag_coef * vel_rf.squaredNorm() * area *
                          Atmosphere::get_density(geod.z());
        Vector3d drag_rf{0, 0, std::copysign(drag_mag, -vel_rf.z())};

        aero_force_rf = normal_force_rf + drag_rf;
    } else {
        aero_force_rf = {0, 0, 0};
    }

    Vector3d net_force_enu = rocket_.r2enu(aero_force_rf + thrust_rf);
    net_force_enu.z() -= (9.81 * mass);

    return net_force_enu;
}

/**
 * @brief Takes in time and angular velocity to calculate net torque
 *
 * @param vel_enu Rocket's velocity with respect to the ENU frame
 * @param ang_vel_enu Rocket's angular velocity with respect to the ENU
 * frame
 * @return Vector3  Vector containing the net torque on the rocket in the x, y,
 * and z directions
 */
Vector3d RungeKutta::calc_net_torque(Vector3d vel_enu, Vector3d pos_enu) {
    /*************** Retrieve Instantaneous Rocket Parameters *****************/

    Vector3d Cp_vect_rf = rocket_.get_Cp_vect();

    double inertia[9];  // moments of inertia
    rocket_.get_I(inertia);

    double area = rocket_.get_A_ref();
    double c_Na = rocket_.get_Cna();  // normal force coefficient derivative
    double drag_coef = rocket_.get_Cd();

    Vector3d geod = ecef2geod(i2ecef(pos_enu));

    /************************ Calculate Net Torque ***************************/

    Vector3d aero_force_rf;
    Vector3d aero_torque_rf;
    Vector3d aero_force_enu;
    Vector3d aero_torque_enu;
    Vector3d net_force_rf;
    Vector3d net_torque_rf;

    if (vel_enu.norm() > 0.01) {
        Vector3d vel_rf = rocket_.enu2r(vel_enu);
        Vector3d normal_force_rf;

        // angle between velocity vector and rocket axis
        double alpha = acos(vel_rf.z() / vel_rf.norm());

        double normal_coef = c_Na * alpha;

        double normal_force_mag = 0.5 * normal_coef * vel_rf.squaredNorm() *
                                  area * Atmosphere::get_density(geod.z());
        normal_force_rf = {(-vel_rf.x()), (-vel_rf.y()), 0};

        normal_force_rf.normalize();
        normal_force_rf = normal_force_rf * normal_force_mag;

        double drag_mag = 0.5 * drag_coef * vel_rf.squaredNorm() * area *
                          Atmosphere::get_density(geod.z());
        Vector3d drag_rf{0, 0, std::copysign(drag_mag, -vel_rf.z())};

        aero_force_rf = normal_force_rf + drag_rf;
        aero_torque_rf = Cp_vect_rf.cross(aero_force_rf);
    } else {
        aero_torque_rf = {0, 0, 0};
    }

    Vector3d net_torque_enu = rocket_.r2enu(aero_torque_rf);

    return net_torque_enu;
}

/**
 * @brief Calculates a possible rocket state based on the initial and inputed
 * state
 *
 * Method performs a basic Euler step on the velocity and angular velocity of
 * the rocket, as well as recalculating forces, accereration, angular
 * acceleration, and orientation, using the initial state of the rocket and the
 * data from the state an inputed state
 *
 * @param k The state of the rocket being used to calculate the next state
 * @return RungeKuttaState  Velocity, accereration, angular velocity, and
 * angular acceleration at a moment
 */
RungeKutta::RungeKuttaState RungeKutta::calc_state(double tStamp, double tStep,
                                                   RungeKuttaState k) {
    // the rocket's initial state (k1), regardless of which state is being
    // calculated
    Vector3d pos_initial = rocket_.get_r_vect();
    Vector3d vel_initial = rocket_.get_r_dot();
    Vector3d ang_vel_initial = rocket_.get_w_vect();
    Quaterniond orient_true = rocket_.get_q_ornt();

    double inertia[9];
    rocket_.get_I(inertia);

    Quaterniond orient = update_quaternion(orient_true, k.ang_vel, tStep);
    rocket_.set_q_ornt(orient);  // sets the orientation to the current state in
                                 // order to calculate net forces
    // Euler Step: y = x + (dx * t)
    // x = initial state of the rocket
    // dx = taken from state k
    Vector3d pos_k = pos_initial + k.vel * tStep;
    Vector3d vel_k = vel_initial + k.accel * tStep;
    Vector3d accel_k =
        calc_net_force(tStamp, k.pos, k.vel) / rocket_.get_mass();
    Vector3d ang_vel_k = ang_vel_initial + (k.ang_accel * tStep);
    Vector3d net_torque_new = calc_net_torque(k.vel, k.pos);
    Vector3d ang_accel_k;
    ang_accel_k.x() = net_torque_new.x() / inertia[0];
    ang_accel_k.y() = net_torque_new.y() / inertia[4];
    ang_accel_k.z() = net_torque_new.z() / inertia[8];

    rocket_.set_q_ornt(orient_true);  // resets the orientation to the initial

    // Set the values of the State structure
    return {pos_k, vel_k, accel_k, ang_vel_k, ang_accel_k};
}

/**
 * @brief Updates the orientation quaternion of the rocket from an angular
 * velocity vector
 *
 * Method creates a rotation quaternion that represents the total rotation the
 * rocket will undergo throughout the particular timestep using the axis-angle
 * method. It then applies this rotation to the current orientation quaterion by
 * left-multiplying it.
 *
 * @param q_ornt    The current orientation quaternion
 * @param omega_enu  The angular velocity vector in ENU frame
 * @param tStep     Simulation time step size
 *
 * @return Quaterniond Updated quaterion with the applied rotation
 */
Quaterniond PhysicsEngine::update_quaternion(Quaterniond q_ornt,
                                             Vector3d omega_enu,
                                             double tStep) const {
    // Calculate half-angle traveled during this timestep
    double half_angle = 0.5 * omega_enu.norm() * tStep;

    // Normalize the axis of rotation before using in axis-angle method
    omega_enu.normalize();

    // Assemble quaternion using axis-angle representation
    Quaterniond q_rotation{cos(half_angle), sin(half_angle) * omega_enu.x(),
                           sin(half_angle) * omega_enu.y(),
                           sin(half_angle) * omega_enu.z()};

    // Apply the rotation to the rocket's orientation quaternion
    q_ornt = q_rotation * q_ornt;

    // Orientation quaternions must always stay at unit norm
    q_ornt.normalize();

    return q_ornt;
}

/**
 * @brief Converts the rocket's ENU coordinates to ECEF
 *
 * Method takes in the position of the rocket in the East-North-Up reference
 * frame and pulls the latitude and longitude of the launchpad from the rocket
 * class.  The launchpad coordinates are converted into radians and used in the
 * transformation matrix.  The transformation matrix and the
 * Earth-Centered-Earth-Fixed (ecef) coordinates of the launchpad are used to
 * calculate the ecef position of the rocket.
 *
 * ENU is the distance East, North, and Up the rocket is from the origin in
 * meters ENU is a flat plane tangent to the earth, with an arbitrary but fixed
 * origin
 *
 * ECEF is the distance the rocket is from the center of mass of the earth
 * The X axis is the prime meridian and 180 degrees longitude, on the equator
 * The Y axis is 90 degrees east and 90 degrees west, on the equator
 * The Z axis is north and south
 *
 * https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
 * "From ENU to ECEF"
 *
 * @param pos_enu Position of the rocket in the East-North-Up reference frame
 * @return Vector3d Position of the rocket in the Earth-Centered-Earth-Fixed
 * reference frame
 */
Vector3d RungeKutta::i2ecef(Vector3d pos_enu) {
    double lambda =
        rocket_.get_launch_geod().y() * M_PI / 180;  // longitude of launchpad
    double lat =
        rocket_.get_launch_geod().x() * M_PI / 180;  // latitude of launchpad
    Vector3d ecef = rocket_.get_r_ecef();

    Eigen::Matrix<double, 3, 3> transform{
        {-std::sin(lambda), -std::sin(lat) * std::cos(lambda),
         std::cos(lat) * std::cos(lambda)},
        {std::cos(lambda), -std::sin(lat) * std::sin(lambda),
         std::cos(lat) * std::sin(lambda)},
        {0, std::cos(lat), std::sin(lat)}};

    ecef = (transform * pos_enu) + rocket_.get_launch_ecef();

    return ecef;
}

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
 * The X axis is the prime meridian and 180 degrees longitude, on the equator
 * The Y axis is 90 degrees east and 90 degrees west, on the equator
 * The Z axis is north and south
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
Vector3d RungeKutta::ecef2geod(Vector3d ecef) {
    Vector3d geod = rocket_.get_r_geod();

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