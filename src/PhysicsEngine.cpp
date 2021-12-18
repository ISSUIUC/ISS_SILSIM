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

#include <math.h>

#include <string>
#include <vector>
#include <iostream>

#include "Rocket.h"
#include "Vector3.h"
#include "quaternion.h"

#define RAD2DEG (180.0 / 3.14159265)

#define PHYSENG_DEBUG

/**
 * @brief Calculates forces and moments and integrates with a simple euler step
 *
 * @param tStamp Current simulation timestamp
 * @param tStep Simulation time step size
 */
void ForwardEuler::march_step(double tStamp, double tStep) {
    /*************** Retrieve instantaneous rocket parameters *****************/

    // Inertial frame dynamics parameters
    static Vector3 r_vect_if = rocket_.get_r_vect();
    static Vector3 r_dot_if = rocket_.get_r_dot();
    static Vector3 r_ddot_if = rocket_.get_r_ddot();
    static Vector3 w_vect_if = rocket_.get_w_vect();
    static Vector3 w_dot_if = rocket_.get_w_dot();
    static Vector3 f_net_if = rocket_.get_f_net();
    static Vector3 t_net_if = rocket_.get_t_net();

    // Quaternion from inertial to rocket frame
    static Quaternion<double> q_ornt = rocket_.get_q_ornt();

    // CG to Cp vector
    static Vector3 Cp_vect_rf = rocket_.get_Cp_vect();

    // Get moment of inertia tenspr
    static double I_tens[9];
    rocket_.get_I(I_tens);

    // Static parameters
    static double mass = rocket_.get_mass();
    // static double d_ref = rocket_.get_d_ref();
    static double A_ref = rocket_.get_A_ref();
    static double Cna = rocket_.get_Cna();
    static double Cd = rocket_.get_Cd();

    // Motor thrust vector, rocket frame
    static Vector3 thrust_rf = motor_.get_thrust(tStamp);

    /********************* Calculate forces and torques ***********************/

    static Vector3 f_aero_rf;  // Aerodynamic forces, rocket frame
    static Vector3 t_aero_rf;  // Aerodynamic torques, rocket frame
    static Vector3 f_aero_if;  // Aerodynamic forces, inertial frame
    static Vector3 t_aero_if;  // Aerodynamic torques, inertial frame

    static Vector3 f_net_rf;
    static Vector3 t_net_rf;

    if (r_dot_if.magnitude() > 0.01) {
        // if (true) {

        Vector3 rocket_axis_rf(0, 0, 1);

        Vector3 v_rf;
        v_rf = rocket_.i2r(r_dot_if);

        // printf("v_rf: <%f, %f, %f>\n", v_rf.x, v_rf.y, v_rf.z);

        double alpha = acos(v_rf.z / v_rf.magnitude());

        Vector3 f_N_rf;

        double f_N_mag = Cna * alpha * 0.5 * 1.225 * v_rf.magnitude2() * A_ref;
        f_N_rf.x = -v_rf.x;
        f_N_rf.y = -v_rf.y;
        f_N_rf.z = 0;
        f_N_rf.normalize();
        f_N_rf = f_N_rf * f_N_mag;

        // printf("f_N_mag: %f\n", f_N_mag);
        // printf("f_N_rf: <%f, %f, %f>\n", f_N_rf.x, f_N_rf.y, f_N_rf.z);

        double f_D_mag = Cd * 0.5 * 1.225 * v_rf.magnitude2();
        Vector3 f_D_rf(0, 0, -f_D_mag);

        // printf("f_D_rf: <%f, %f, %f>\n", f_D_rf.x, f_D_rf.y, f_D_rf.z);

        f_aero_rf = f_N_rf + f_D_rf;

        // printf("f_aero_rf: <%f, %f, %f>\n\n", f_aero_rf.x, f_aero_rf.y,
        // f_aero_rf.z);

        // t_aero_rf = f_aero_rf.cross(Cp_vect_rf);
        t_aero_rf = Cp_vect_rf.cross(f_aero_rf);

    } else {
        f_aero_rf.x = 0;
        f_aero_rf.y = 0;
        f_aero_rf.z = 0;

        t_aero_rf.x = 0;
        t_aero_rf.y = 0;
        t_aero_rf.z = 0;
    }

    f_net_if = rocket_.r2i(f_aero_rf + thrust_rf);
    f_net_if.z -= mass * 9.81;

    t_net_if = rocket_.r2i(t_aero_rf);

    /************************** Perform euler step ****************************/

    r_vect_if += r_dot_if * tStep;
    r_dot_if += r_ddot_if * tStep;
    r_ddot_if = f_net_if / mass;

    // Assemble instantaneous rotation quaternion
    double w_mag = w_vect_if.magnitude();
    if (w_mag > 0.000001) {
        Quaternion<double> q_rot;
        q_rot.Set(cos(w_mag / 2.0), (w_vect_if.x / w_mag) * sin(w_mag / 2.0),
                  (w_vect_if.y / w_mag) * sin(w_mag / 2.0),
                  (w_vect_if.z / w_mag) * sin(w_mag / 2.0));

        // Apply instantaneous rotation
        q_ornt = q_ornt * q_rot;
        q_ornt.Normalize();
    }

    w_vect_if += w_dot_if * tStep;

    w_dot_if.x = t_net_if.x / I_tens[0];
    w_dot_if.y = t_net_if.y / I_tens[4];
    w_dot_if.z = t_net_if.z / I_tens[8];

    // Naively accounting for launch rail
    if (r_vect_if.magnitude() < 4.50) {
        w_dot_if.x = 0;
        w_dot_if.y = 0;
        w_dot_if.z = 0;
        w_vect_if.x = 0;
        w_vect_if.y = 0;
        w_vect_if.z = 0;
    }

    rocket_.set_f_net(f_net_if);
    rocket_.set_t_net(t_net_if);

    rocket_.set_r_vect(r_vect_if);
    rocket_.set_r_dot(r_dot_if);
    rocket_.set_r_ddot(r_ddot_if);

    rocket_.set_q_ornt(q_ornt);

    rocket_.set_w_vect(w_vect_if);
    rocket_.set_w_dot(w_dot_if);
}

void ForwardEulerRefactor::march_step(double tStamp, double tStep) {

    // {variable}_rf = rocket frame (stuck to rocket)
    // {variable}_if = inertial frame (stuck to earth)
    // f_{variable} = force
    // t_{variable} = torque

    /*************** Retrieve instantaneous rocket parameters *****************/

    // Inertial frame dynamics parameters
    Vector3 r_vect_if = rocket_.get_r_vect(); // position
    Vector3 r_dot_if = rocket_.get_r_dot();   // velocity
    Vector3 r_ddot_if = rocket_.get_r_ddot(); // acceleration
    Vector3 w_vect_if = rocket_.get_w_vect(); // angular velocity (omega)
    Vector3 w_dot_if = rocket_.get_w_dot();   // angular acceleration
    Vector3 f_net_if = rocket_.get_f_net();   // net force (Newtons)
    Vector3 t_net_if = rocket_.get_t_net();   // net torque (Newtons*meters)

    // Quaternion from inertial to rocket frame
    Quaternion<double> q_ornt = rocket_.get_q_ornt();  // orientation of rocket

    // CG to Cp vector
    Vector3 Cp_vect_rf = rocket_.get_Cp_vect();   // CG to Cp (center of pressure) vector

    // Get moment of inertia tenspr
    double I_tens[9];
    rocket_.get_I(I_tens);  // moment of inertia

    // parameters
    double mass = rocket_.get_mass();       // mass of rocket
    double A_ref = rocket_.get_A_ref();     // 0.0194; ref area in m^2
    double c_Na = rocket_.get_Cna();         // 9.65; normal force coefficient derivative
    double c_D = rocket_.get_Cd();           // 0.630; drag coefficient

    // Motor thrust vector, rocket frame
    Vector3 thrust_rf = motor_.get_thrust(tStamp);  // thrust of rocket at current timestamp

    /********************* Calculate forces and torques ***********************/
    Vector3 f_aero_rf;  // Aerodynamic forces, rocket frame
    Vector3 t_aero_rf;  // Aerodynamic torques, rocket frame
    Vector3 f_aero_if;  // Aerodynamic forces, inertial frame
    Vector3 t_aero_if;  // Aerodynamic torques, inertial frame

    Vector3 f_net_rf;  // net force
    Vector3 t_net_rf;  // net torque

    // Set aerodynamic forces and torques to zero if velocity is small. This 
    // avoids calculations returning NaN values.
    if (r_dot_if.magnitude() > 0.01) {

        Vector3 rocket_axis_rf(0, 0, 1);
        Vector3 v_rf = rocket_.i2r(r_dot_if);
        double alpha = acos(v_rf.z / v_rf.magnitude());  // angle between velocity vector and rocket axis
        Vector3 f_N_rf;  // normal aerodynamic force

        double c_N = c_Na * alpha;
        double f_N_mag = c_N * 0.5 * 1.225 * v_rf.magnitude2() * A_ref;  // magnitude of normal force (assuming constant density (will change))

        f_N_rf.x = (-v_rf.x);
        f_N_rf.y = (-v_rf.y);
        f_N_rf.z = 0;

        f_N_rf.normalize();
        f_N_rf = f_N_rf * f_N_mag;

        double f_D_mag = c_D * 0.5 * 1.225 * v_rf.magnitude2() * A_ref;  // Ask about A_ref
        Vector3 f_D_rf(0, 0, -f_D_mag);

        f_aero_rf = f_N_rf + f_D_rf;
        t_aero_rf = Cp_vect_rf.cross(f_aero_rf);

    } else {
        f_aero_rf.x = 0;
        f_aero_rf.y = 0;
        f_aero_rf.z = 0;

        t_aero_rf.x = 0;
        t_aero_rf.y = 0;
        t_aero_rf.z = 0;
    }

    f_net_if = rocket_.r2i(f_aero_rf + thrust_rf);
    f_net_if.z -= 9.81 * mass;

    t_net_if = rocket_.r2i(t_aero_rf);

    /************************** Perform euler step ****************************/

    r_vect_if += r_dot_if * tStep;
    r_dot_if += r_ddot_if * tStep;
    r_ddot_if = f_net_if / mass;

    double w_mag = w_vect_if.magnitude();
    if (w_mag > 0.000001) {
        Quaternion<double> q_rot;
        q_rot.Set(cos(w_mag / 2.0), (w_vect_if.x / w_mag) * sin(w_mag / 2.0),
                  (w_vect_if.y / w_mag) * sin(w_mag / 2.0),
                  (w_vect_if.z / w_mag) * sin(w_mag / 2.0));

        // Apply instantaneous rotation
        q_ornt = q_ornt * q_rot;
        q_ornt.Normalize();
    }

    w_vect_if += w_dot_if * tStep;

    w_dot_if.x = t_net_if.x / I_tens[0];
    w_dot_if.y = t_net_if.y / I_tens[4];
    w_dot_if.z = t_net_if.z / I_tens[8];

    // Naively accounting for launch rail
    if (r_vect_if.magnitude() < 4.50) {
        w_dot_if.x = 0;
        w_dot_if.y = 0;
        w_dot_if.z = 0;
        w_vect_if.x = 0;
        w_vect_if.y = 0;
        w_vect_if.z = 0;
    }

#ifdef PHYSENG_DEBUG
    printf("############### PHYSENG_DEBUG ###############\n");
    printf("TimestampL %f\n", tStamp);
    printf("thrust_rf = <%f, %f, %f>\n", thrust_rf.x, thrust_rf.y, thrust_rf.z);
    printf("f_aero_rf = <%f, %f, %f>\t", f_aero_rf.x, f_aero_rf.y, f_aero_rf.z);
    printf("t_aero_rf = <%f, %f, %f>\n", t_aero_rf.x, t_aero_rf.y, t_aero_rf.z);
    printf("t_net_rf = <%f, %f, %f>\t", t_net_rf.x, t_net_rf.y, t_net_rf.z);
    printf("f_net_if = <%f, %f, %f>\n", f_net_if.x, f_net_if.y, f_net_if.z);
    printf("r_dot_if = <%f, %f, %f>\t", r_dot_if.x, r_dot_if.y, r_dot_if.z);
    printf("r_ddot_if = <%f, %f, %f>\n", r_ddot_if.x, r_ddot_if.y, r_ddot_if.z);
    printf("\n");
#endif

    rocket_.set_r_vect(r_vect_if);
    rocket_.set_r_dot(r_dot_if);
    rocket_.set_r_ddot(r_ddot_if);
    rocket_.set_w_vect(w_vect_if);
    rocket_.set_w_dot(w_dot_if);
    rocket_.set_f_net(f_net_if);
    rocket_.set_t_net(t_net_if);
    rocket_.set_q_ornt(q_ornt);
}
