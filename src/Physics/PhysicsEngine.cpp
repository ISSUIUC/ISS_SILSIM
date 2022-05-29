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

#include "Atmosphere.h"

using Eigen::Quaterniond;
using Eigen::Vector3d;

#define RAD2DEG (180.0 / 3.14159265)

constexpr double kFeetToMeters = 0.3048;

ForwardEuler::ForwardEuler(Rocket& rocket, RocketMotor& motor)
    : PhysicsEngine(rocket, motor) {
    euler_logger =
        spdlog::basic_logger_mt("Euler_Logger", "logs/forward_euler.log");
}

/*****************************************************************************/
/*                         PHYSICS ENGINE FUNCTIONS                          */
/*****************************************************************************/

/**
 * @brief Calculates the net force and net moment acting on the rocket in the
 * rocket body frame
 *
 * @param tStamp Current time stamp in the simulation
 * @param pos_enu Rocket's current position in the ENU frame
 * @param vel_enu Rocket's current velocity in the ENU frame
 * @return Vector3d,Vector3d The net force and net moment vectors acting on the
 * rocket in the Rocket body frame
 */
std::pair<Vector3d, Vector3d> PhysicsEngine::calc_forces_and_moments(
    double tStamp, Vector3d pos_enu, Vector3d vel_enu) {
    // {variable}_rf = rocket frame (stuck to rocket)
    // {variable}_enu = ENU frame (stuck to earth)

    /*************** Retrieve Instantaneous Rocket Parameters *****************/

    Vector3d thrust_rf = motor_.get_thrust_vector(tStamp);
    Vector3d cp_vect_rf = rocket_.get_cp_vect();

    double total_mass = rocket_.get_total_mass();
    double area = rocket_.get_reference_area();
    double CN = rocket_.get_total_normal_force_coeff();
    double CA = rocket_.get_total_axial_force_coeff();

    // enu2r pulls a quaternion from the rocket, be sure to set orientation
    // beforehand
    Vector3d vel_rf = rocket_.enu2r(vel_enu);
    double velocity_magnitude = vel_rf.squaredNorm();

    Vector3d geod = rocket_.ecef2geod(rocket_.position_enu2ecef(pos_enu));

    double altitude = Atmosphere::get_density(geod.z());

    /************************* Calculate Net Force ****************************/
    Vector3d aero_force_rf;

    if (vel_enu.norm() > 0.01) {
        Vector3d normal_force_rf;

        double normal_force_mag =
            0.5 * CN * velocity_magnitude * area * altitude;
        normal_force_rf = {(-vel_rf.x()), (-vel_rf.y()), 0};

        normal_force_rf.normalize();
        normal_force_rf = normal_force_rf * normal_force_mag;

        double axial_force_mag =
            0.5 * CA * velocity_magnitude * area * altitude;
        Vector3d axial_force_rf{0, 0,
                                std::copysign(axial_force_mag, -vel_rf.z())};

        aero_force_rf = normal_force_rf + axial_force_rf;
    } else {
        aero_force_rf = {0, 0, 0};
    }

    Vector3d grav_rf = rocket_.gravity_vector_rf() * 9.81 * total_mass;
    Vector3d net_force_rf = aero_force_rf + thrust_rf + grav_rf;

    /************************ Calculate Net Torque ***************************/
    Vector3d aero_moment_rf;

    if (vel_enu.norm() > 0.01) {
        double normal_force_mag =
            0.5 * CN * velocity_magnitude * area * altitude;
        Vector3d normal_force_rf = {(-vel_rf.x()), (-vel_rf.y()), 0};
        normal_force_rf.normalize();
        normal_force_rf = normal_force_rf * normal_force_mag;

        double axial_force_mag =
            0.5 * CA * velocity_magnitude * area * altitude;
        Vector3d axial_force_rf{0, 0,
                                std::copysign(axial_force_mag, -vel_rf.z())};
        Vector3d aero_force_rf = normal_force_rf + axial_force_rf;
        aero_moment_rf = cp_vect_rf.cross(aero_force_rf);

    } else {
        aero_moment_rf = {0, 0, 0};
    }

    Vector3d net_moment_rf = aero_moment_rf;

    return {net_force_rf, net_moment_rf};
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

/*****************************************************************************/
/*                         FORWARD EULER FUNCTIONS                           */
/*****************************************************************************/

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
    // t_{variable} = moment

    /*************** Retrieve instantaneous rocket parameters *****************/

    // ENU frame dynamics parameters
    Vector3d pos_enu = rocket_.get_r_vect();       // position
    Vector3d vel_enu = rocket_.get_r_dot();        // velocity
    Vector3d accel_enu = rocket_.get_r_ddot();     // acceleration
    Vector3d ang_vel_enu = rocket_.get_w_vect();   // angular velocity (omega)
    Vector3d ang_accel_enu = rocket_.get_w_dot();  // angular acceleration

    // Quaternion from ENU to rocket frame
    Quaterniond q_ornt = rocket_.get_q_ornt();

    // Aerodynamic and Inertial parameters
    std::array<double, 9> I_tens = rocket_.get_I();  // moment of inertia
    double total_mass = rocket_.get_total_mass();    // total mass of rocket
    double alpha = rocket_.get_alpha();
    double mach = rocket_.get_mach();

    std::pair<Vector3d, Vector3d> force_and_moment =
        calc_forces_and_moments(tStamp, pos_enu, vel_enu);
    Vector3d net_force_rf = force_and_moment.first;
    Vector3d net_moment_rf = force_and_moment.second;

    Vector3d net_force_enu = rocket_.r2enu(net_force_rf);
    Vector3d net_moment_enu = rocket_.r2enu(net_moment_rf);

    /************************** Perform euler step ****************************/

    pos_enu += vel_enu * tStep;
    vel_enu += accel_enu * tStep;
    accel_enu = net_force_enu / total_mass;

    q_ornt = update_quaternion(q_ornt, ang_vel_enu, tStep);

    ang_vel_enu += ang_accel_enu * tStep;

    Vector3d ang_accel_rf{net_moment_rf.x() / I_tens[0],
                          net_moment_rf.y() / I_tens[4],
                          net_moment_rf.z() / I_tens[8]};

    ang_accel_enu = rocket_.r2enu(ang_accel_rf);

    // Naively accounting for launch rail
    if (pos_enu.norm() < (17 * kFeetToMeters)) {
        ang_accel_enu = {0, 0, 0};
        ang_vel_enu = {0, 0, 0};
    }

    // Do not calculate rocket's angle-of-attack and mach number if velocity is
    // small to avoid NaN values
    if (vel_enu.norm() > 0.01) {
        Vector3d v_rf = rocket_.enu2r(vel_enu);
        alpha = acos(v_rf.z() / v_rf.norm());
        mach = vel_enu.norm() / Atmosphere::get_speed_of_sound(pos_enu.z());
    }

    /*
    euler_logger->debug("Timestamp {}", tStamp);
    euler_logger->debug("thrust_rf = <{}, {}, {}>", thrust_rf.x(),
                        thrust_rf.y(), thrust_rf.z());
    euler_logger->debug("f_aero_rf = <{}, {}, {}>", f_aero_rf.x(),
                        f_aero_rf.y(), f_aero_rf.z());
    euler_logger->debug("t_aero_rf = <{}, {}, {}>", t_aero_rf.x(),
                        t_aero_rf.y(), t_aero_rf.z());
    euler_logger->debug("m_net_rf = <{}, {}, {}>", m_net_rf.x(), m_net_rf.y(),
                        m_net_rf.z());
    euler_logger->debug("f_net_enu = <{}, {}, {}>", f_net_enu.x(),
                        f_net_enu.y(), f_net_enu.z());
    euler_logger->debug("r_dot_enu = <{}, {}, {}>", r_dot_enu.x(),
                        r_dot_enu.y(), r_dot_enu.z());
    euler_logger->debug("r_ddot_enu = <{}, {}, {}>", r_ddot_enu.x(),
                        r_ddot_enu.y(), r_ddot_enu.z());
    */

    rocket_.set_alpha(alpha);
    rocket_.set_mach(mach);
    rocket_.set_r_vect(pos_enu);
    rocket_.set_r_dot(vel_enu);
    rocket_.set_r_ddot(accel_enu);
    rocket_.set_w_vect(ang_vel_enu);
    rocket_.set_w_dot(ang_accel_enu);
    rocket_.set_f_net(net_force_enu);
    rocket_.set_m_net(net_moment_enu);
    rocket_.set_q_ornt(q_ornt);
}

/*****************************************************************************/
/*                          RUNGE KUTTA FUNCTIONS                            */
/*****************************************************************************/

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

    std::array<double, 9> inertia = rocket_.get_I();  // moment of inertia
    double total_mass = rocket_.get_total_mass();
    double alpha = rocket_.get_alpha();
    double mach = rocket_.get_mach();

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

    std::pair<Vector3d, Vector3d> force_and_moment =
        calc_forces_and_moments(tStamp, pos_enu, vel_enu);
    Vector3d net_force_rf = force_and_moment.first;
    Vector3d net_moment_rf = force_and_moment.second;

    Vector3d net_force_enu = rocket_.r2enu(net_force_rf);
    Vector3d net_moment_enu = rocket_.r2enu(net_moment_rf);

    accel_enu = net_force_enu / total_mass;

    ang_vel_enu += tStep * ang_accel_avg;

    Vector3d ang_accel_rf{net_moment_rf.x() / inertia[0],
                          net_moment_rf.y() / inertia[4],
                          net_moment_rf.z() / inertia[8]};
    ang_accel_enu = rocket_.r2enu(ang_accel_rf);

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

    // Do not calculate rocket's angle-of-attack and mach number if velocity is
    // small to avoid NaN values
    if (vel_enu.norm() > 0.01) {
        Vector3d vel_rf = rocket_.enu2r(vel_enu);
        alpha = acos(vel_rf.z() / vel_rf.norm());
        mach = vel_enu.norm() / Atmosphere::get_speed_of_sound(pos_enu.z());
    }

    //---- Set Values ----
    rocket_.set_alpha(alpha);
    rocket_.set_mach(mach);
    rocket_.set_r_vect(pos_enu);
    rocket_.set_r_dot(vel_enu);
    rocket_.set_r_ddot(accel_enu);
    rocket_.set_w_vect(ang_vel_enu);
    rocket_.set_w_dot(ang_accel_enu);
    rocket_.set_f_net(net_force_enu);
    rocket_.set_m_net(net_moment_enu);
    rocket_.set_q_ornt(orient);
}

/**
 * @brief Calculates a possible rocket state based on the initial and inputed
 * state
 *
 * Method performs a basic Euler step on the velocity and angular velocity of
 * the rocket, as well as recalculating forces, acceleration, angular
 * acceleration, and orientation, using the initial state of the rocket and the
 * data from the state an inputed state
 *
 * @param k The state of the rocket being used to calculate the next state
 * @return RungeKuttaState  Velocity, acceleration, angular velocity, and
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

    std::array<double, 9> inertia = rocket_.get_I();

    Quaterniond orient = update_quaternion(orient_true, k.ang_vel, tStep);
    rocket_.set_q_ornt(orient);  // sets the orientation to the current state in
                                 // order to calculate net forces

    std::pair<Vector3d, Vector3d> force_and_moment =
        calc_forces_and_moments(tStamp, k.pos, k.vel);
    Vector3d net_force_rf = force_and_moment.first;
    Vector3d net_moment_rf = force_and_moment.second;
    Vector3d net_force_enu = rocket_.r2enu(net_force_rf);

    // Euler Step: y = x + (dx * t)
    // x = initial state of the rocket
    // dx = taken from state k
    Vector3d pos_k = pos_initial + k.vel * tStep;
    Vector3d vel_k = vel_initial + k.accel * tStep;
    Vector3d accel_k = net_force_enu / rocket_.get_total_mass();
    Vector3d ang_vel_k = ang_vel_initial + (k.ang_accel * tStep);
    Vector3d ang_accel_k{net_moment_rf.x() / inertia[0],
                         net_moment_rf.y() / inertia[4],
                         net_moment_rf.z() / inertia[8]};

    rocket_.set_q_ornt(orient_true);  // resets the orientation to the initial

    // Set the values of the State structure
    return {pos_k, vel_k, accel_k, ang_vel_k, ang_accel_k};
}
