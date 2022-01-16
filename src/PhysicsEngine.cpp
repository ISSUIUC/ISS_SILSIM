/**
 * @file        PhysicsEngine.cpp
 * @authors     Ayberk Yaraneri
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
#include <memory>

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
    // {variable}_if = inertial frame (stuck to earth)
    // f_{variable} = force
    // t_{variable} = torque

    /*************** Retrieve instantaneous rocket parameters *****************/

    // Inertial frame dynamics parameters
    Vector3d r_vect_if = rocket_.get_r_vect();  // position
    Vector3d r_dot_if = rocket_.get_r_dot();    // velocity
    Vector3d r_ddot_if = rocket_.get_r_ddot();  // acceleration
    Vector3d w_vect_if = rocket_.get_w_vect();  // angular velocity (omega)
    Vector3d w_dot_if = rocket_.get_w_dot();    // angular acceleration
    Vector3d f_net_if = rocket_.get_f_net();    // net force (Newtons)
    Vector3d t_net_if = rocket_.get_t_net();    // net torque (Newtons*meters)

    // Quaternion from inertial to rocket frame
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
    Vector3d f_aero_rf;  // Aerodynamic forces, rocket frame
    Vector3d t_aero_rf;  // Aerodynamic torques, rocket frame
    Vector3d f_aero_if;  // Aerodynamic forces, inertial frame
    Vector3d t_aero_if;  // Aerodynamic torques, inertial frame

    Vector3d f_net_rf;  // net force
    Vector3d t_net_rf;  // net torque

    // Set aerodynamic forces and torques to zero if velocity is small. This
    // avoids calculations returning NaN values.
    if (r_dot_if.norm() > 0.01) {
        Vector3d rocket_axis_rf(0, 0, 1);
        Vector3d v_rf = rocket_.i2r(r_dot_if);
        double alpha =
            acos(v_rf.z() /
                 v_rf.norm());  // angle between velocity vector and rocket axis
        Vector3d f_N_rf;        // normal aerodynamic force

        double c_N = c_Na * alpha;
        double f_N_mag = c_N * 0.5 * Atmosphere::get_density(r_vect_if.z()) *
                         v_rf.squaredNorm() *
                         A_ref;  // norm of normal force (assuming constant
                                 // 0.5 is a coefficient in the equation

        f_N_rf.x() = (-v_rf.x());
        f_N_rf.y() = (-v_rf.y());
        f_N_rf.z() = 0;

        f_N_rf.normalize();
        f_N_rf = f_N_rf * f_N_mag;

        double f_D_mag = c_D * 0.5 * Atmosphere::get_density(r_vect_if.z()) *
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

    f_net_if = rocket_.r2i(f_aero_rf + thrust_rf);
    f_net_if.z() -= (9.81 * mass);

    t_net_if = rocket_.r2i(t_aero_rf);

    /************************** Perform euler step ****************************/

    r_vect_if += r_dot_if * tStep;
    r_dot_if += r_ddot_if * tStep;
    r_ddot_if = f_net_if / mass;

    q_ornt = update_quaternion(q_ornt, w_vect_if, tStep);

    w_vect_if += w_dot_if * tStep;

    w_dot_if.x() = t_net_if.x() / I_tens[0];
    w_dot_if.y() = t_net_if.y() / I_tens[4];
    w_dot_if.z() = t_net_if.z() / I_tens[8];

    // Naively accounting for launch rail
    if (r_vect_if.norm() < 4.50) {
        w_dot_if.x() = 0;
        w_dot_if.y() = 0;
        w_dot_if.z() = 0;
        w_vect_if.x() = 0;
        w_vect_if.y() = 0;
        w_vect_if.z() = 0;
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
    euler_logger->debug("f_net_if = <{}, {}, {}>", f_net_if.x(), f_net_if.y(),
                        f_net_if.z());
    euler_logger->debug("r_dot_if = <{}, {}, {}>", r_dot_if.x(), r_dot_if.y(),
                        r_dot_if.z());
    euler_logger->debug("r_ddot_if = <{}, {}, {}>", r_ddot_if.x(),
                        r_ddot_if.y(), r_ddot_if.z());

    rocket_.set_r_vect(r_vect_if);
    rocket_.set_r_dot(r_dot_if);
    rocket_.set_r_ddot(r_ddot_if);
    rocket_.set_w_vect(w_vect_if);
    rocket_.set_w_dot(w_dot_if);
    rocket_.set_f_net(f_net_if);
    rocket_.set_t_net(t_net_if);
    rocket_.set_q_ornt(q_ornt);
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
 * @param omega_if  The angular velocity vector in inertial frame
 * @param tStep     Simulation time step size
 *
 * @return Quaternion<double> Updated quaterion with the applied rotation
 */
Quaterniond ForwardEuler::update_quaternion(Quaterniond q_ornt,
                                            Vector3d omega_if,
                                            double tStep) const {
    // Calculate half-angle traveled during this timestep
    double half_angle = 0.5 * omega_if.norm() * tStep;

    // Normalize the axis of rotation before using in axis-angle method
    omega_if.normalize();

    // Assemble quaternion using axis-angle representation
    Quaterniond q_rotation{cos(half_angle), sin(half_angle) * omega_if.x(),
                           sin(half_angle) * omega_if.y(),
                           sin(half_angle) * omega_if.z()};

    // Apply the rotation to the rocket's orientation quaternion
    q_ornt = q_rotation * q_ornt;

    // Orientation quaternions must always stay at unit norm
    q_ornt.normalize();

    return q_ornt;
}
