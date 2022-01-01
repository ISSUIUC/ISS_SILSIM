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

#include <cmath>

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
    // {variable}_rf = rocket frame (stuck to rocket)
    // {variable}_if = inertial frame (stuck to earth)
    // f_{variable} = force
    // t_{variable} = torque

    /*************** Retrieve instantaneous rocket parameters *****************/

    // Inertial frame dynamics parameters
    Vector3 r_vect_if = rocket_.get_r_vect();  // position
    Vector3 r_dot_if = rocket_.get_r_dot();    // velocity
    Vector3 r_ddot_if = rocket_.get_r_ddot();  // acceleration
    Vector3 w_vect_if = rocket_.get_w_vect();  // angular velocity (omega)
    Vector3 w_dot_if = rocket_.get_w_dot();    // angular acceleration
    Vector3 f_net_if = rocket_.get_f_net();    // net force (Newtons)
    Vector3 t_net_if = rocket_.get_t_net();    // net torque (Newtons*meters)

    // Quaternion from inertial to rocket frame
    Quaternion<double> q_ornt = rocket_.get_q_ornt();  // orientation of rocket

    // CG to Cp vector
    Vector3 Cp_vect_rf =
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
    Vector3 thrust_rf =
        motor_.get_thrust(tStamp);  // thrust of rocket at current timestamp

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
        double alpha = acos(
            v_rf.z /
            v_rf.magnitude());  // angle between velocity vector and rocket axis
        Vector3 f_N_rf;         // normal aerodynamic force

        double c_N = c_Na * alpha;
        double f_N_mag = c_N * 0.5 * 1.225 * v_rf.magnitude2() *
                         A_ref;  // magnitude of normal force (assuming constant
                                 // density of 1.225 (will change))
                                 // 0.5 is a coefficient in the equation

        f_N_rf.x = (-v_rf.x);
        f_N_rf.y = (-v_rf.y);
        f_N_rf.z = 0;

        f_N_rf.normalize();
        f_N_rf = f_N_rf * f_N_mag;

        double f_D_mag = c_D * 0.5 * 1.225 * v_rf.magnitude2() * A_ref;
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
    f_net_if.z -= (9.81 * mass);

    t_net_if = rocket_.r2i(t_aero_rf);

    /************************** Perform euler step ****************************/

    r_vect_if += r_dot_if * tStep;
    r_dot_if += r_ddot_if * tStep;
    r_ddot_if = f_net_if / mass;

    double w_mag = w_vect_if.magnitude();
    if (w_mag > 0.000001) {  // if the rocket is moving; numbers below this
                             // threshold are interpreted as 0 and cause errors
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

void RungeKutta::march_step(double tStamp, double tStep) {

    /*************** Retrieve Instantaneous Rocket Parameters *****************/

    // {variable}_rf = rocket frame (stuck to rocket)
    // {variable}_if = inertial frame (stuck to earth)

    Vector3 pos_if = rocket_.get_r_vect();
    Vector3 vel_if = rocket_.get_r_dot();
    Vector3 accel_if = rocket_.get_r_ddot();
    Vector3 ang_vel_if = rocket_.get_w_vect();
    Vector3 ang_accel_if = rocket_.get_w_dot();
    Vector3 net_force_if = rocket_.get_f_net();
    Vector3 net_torque_if = rocket_.get_t_net();

    Vector3 Cp_vect_rf = rocket_.get_Cp_vect();
    Vector3 thrust_rf = motor_.get_thrust(tStamp);
    
    Quaternion<double> orient = rocket_.get_q_ornt();

    double inertia[9];         // moments of inertia
    rocket_.get_I(inertia);

    double mass = rocket_.get_mass();
    double area = rocket_.get_A_ref();
    double c_Na = rocket_.get_Cna();  // normal force coefficient derivative
    double drag_coef = rocket_.get_Cd();

    /********************* Calculate Forces and Torques ***********************/

    Vector3 aero_force_rf;
    Vector3 aero_torque_rf;
    Vector3 aero_force_if;
    Vector3 aero_torque_if;
    Vector3 net_force_rf;
    Vector3 net_torque_rf;

    if (vel_if.magnitude() > 0.01) {
        Vector3 rocket_axis_rf(0, 0, 1);
        Vector3 vel_rf = rocket_.i2r(vel_if);
        Vector3 normal_force_rf;

        double alpha = acos(vel_rf.z / vel_rf.magnitude());  // angle between velocity vector and rocket axis
        double normal_coef = c_Na * alpha;
        
        // 1.225 is temporary density of air
        double normal_force_mag = 0.5 * normal_coef * vel_rf.magnitude2() * area * 1.225;
        normal_force_rf.x = (-vel_rf.x);
        normal_force_rf.y = (-vel_rf.y);
        normal_force_rf.z = 0;

        normal_force_rf.normalize();
        normal_force_rf = normal_force_rf * normal_force_mag;

        double drag_mag = 0.5 * drag_coef * vel_rf.magnitude2() * area * 1.225;
        Vector3 drag_rf(0, 0, -(drag_mag));

        aero_force_rf = normal_force_rf + drag_rf;
        aero_torque_rf = Cp_vect_rf.cross(aero_force_rf);
    } else {
        aero_force_rf.x = 0;
        aero_force_rf.y = 0;
        aero_force_rf.z = 0;

        aero_torque_rf.x = 0;
        aero_torque_rf.y = 0;
        aero_torque_rf.z = 0;
    }

    net_force_if = rocket_.r2i(aero_force_rf + thrust_rf);
    net_force_if.z -= (9.81 * mass);
    net_torque_if = rocket_.r2i(aero_torque_rf);

    /*************************** Calculate Slopes *****************************/

    Quaternion<double> orient_true = rocket_.get_q_ornt();

    //---- k1 ----
    Vector3 vel_k1 = vel_if;
    Vector3 accel_k1 = accel_if;
    Vector3 ang_vel_k1 = ang_vel_if;
    Vector3 ang_accel_k1 = ang_accel_if;

    //---- k2 ----
    Vector3 vel_k2 = vel_k1 + (accel_k1 * tStep * 0.5);
    Vector3 accel_k2 = calc_net_force(tStamp, vel_k1) / mass;
    Vector3 ang_vel_k2 = ang_vel_k1 + (ang_accel_k1 * tStep * 0.5);
    Vector3 ang_accel_k2;
    Vector3 net_torque_new = calc_net_torque(tStamp, vel_k1, ang_vel_k1);
    ang_accel_k2.x = net_torque_new.x / inertia[0];
    ang_accel_k2.y = net_torque_new.y / inertia[4];
    ang_accel_k2.z = net_torque_new.z / inertia[8];
    
    //---- k3 ----
    double ang_vel_mag = ang_vel_k2.magnitude();
    if (ang_vel_mag > 0.000001) {
        Quaternion<double> rotation;
        rotation.Set(cos(ang_vel_mag / 2.0), (ang_vel_k2.x / ang_vel_mag) * sin(ang_vel_mag / 2.0),
                  (ang_vel_k2.y / ang_vel_mag) * sin(ang_vel_mag / 2.0),
                  (ang_vel_k2.z / ang_vel_mag) * sin(ang_vel_mag / 2.0));
        orient = orient * rotation;
        orient.Normalize();
    }
    rocket_.set_q_ornt(orient);

    Vector3 vel_k3 = vel_k1 + (accel_k2 * tStep * 0.5);
    Vector3 accel_k3 = calc_net_force(tStamp, vel_k2) / mass;
    Vector3 ang_vel_k3 = ang_vel_k1 + (ang_accel_k2 * tStep * 0.5);
    Vector3 ang_accel_k3;
    net_torque_new = calc_net_torque(tStamp, vel_k2, ang_vel_k2);
    ang_accel_k3.x = net_torque_new.x / inertia[0];
    ang_accel_k3.y = net_torque_new.y / inertia[4];
    ang_accel_k3.z = net_torque_new.z / inertia[8];

    rocket_.set_q_ornt(orient_true);
    
    //---- k4 ----
    ang_vel_mag = ang_vel_k3.magnitude();
    if (ang_vel_mag > 0.000001) {
        Quaternion<double> rotation;
        rotation.Set(cos(ang_vel_mag / 2.0), (ang_vel_k3.x / ang_vel_mag) * sin(ang_vel_mag / 2.0),
                  (ang_vel_k3.y / ang_vel_mag) * sin(ang_vel_mag / 2.0),
                  (ang_vel_k3.z / ang_vel_mag) * sin(ang_vel_mag / 2.0));
        orient = orient * rotation;
        orient.Normalize();
    }
    rocket_.set_q_ornt(orient);

    Vector3 vel_k4 = vel_k1 + (accel_k3 * tStep);
    Vector3 accel_k4 = calc_net_force(tStamp, vel_k3) / mass;
    Vector3 ang_vel_k4 = ang_vel_k1 + (ang_accel_k3 * tStep);
    Vector3 ang_accel_k4;
    net_torque_new = calc_net_torque(tStamp, vel_k3, ang_vel_k3);
    ang_accel_k4.x = net_torque_new.x / inertia[0];
    ang_accel_k4.y = net_torque_new.y / inertia[4];
    ang_accel_k4.z = net_torque_new.z / inertia[8];

    rocket_.set_q_ornt(orient_true);

    /********************** Perform Runge-Kutta Method ************************/

    Vector3 vel_avg = (vel_k1 + (2 * vel_k2) + (2 * vel_k3) + vel_k4) / 6;
    Vector3 accel_avg = (accel_k1 + (2 * accel_k2) + (2 * accel_k3) + accel_k4) / 6;
    Vector3 ang_vel_avg = (ang_vel_k1 + (2 * ang_vel_k2) + (2 * ang_vel_k3) + ang_vel_k4) / 6;
    Vector3 ang_accel_avg = (ang_accel_k1 + (2 * ang_accel_k2) + (2 * ang_accel_k3) + ang_accel_k4) / 6;

    net_force_if = calc_net_force(tStamp, vel_avg);
    net_torque_if = calc_net_torque(tStamp, vel_avg, ang_vel_avg);

    pos_if += tStep * vel_avg;
    vel_if += tStep * accel_avg;
    accel_if = net_force_if / mass;

    ang_vel_if += tStep * ang_accel_avg;
    ang_accel_if.x = net_torque_if.x / inertia[0];
    ang_accel_if.y = net_torque_if.y / inertia[4];
    ang_accel_if.z = net_torque_if.z / inertia[8];

    

    //---- Orientation ----
    ang_vel_mag = ang_vel_avg.magnitude();
    if (ang_vel_mag > 0.000001) {
        Quaternion<double> rotation;
        rotation.Set(cos(ang_vel_mag / 2.0), (ang_vel_avg.x / ang_vel_mag) * sin(ang_vel_mag / 2.0),
                (ang_vel_avg.y / ang_vel_mag) * sin(ang_vel_mag / 2.0),
                (ang_vel_avg.z / ang_vel_mag) * sin(ang_vel_mag / 2.0));
        orient = orient * rotation;
        orient.Normalize();
    }

    //---- Lauch Rail ----
    if (pos_if.magnitude() < 4.50) {
        ang_vel_if.x = 0;
        ang_vel_if.y = 0;
        ang_vel_if.z = 0;
        ang_accel_if.x = 0;
        ang_accel_if.y = 0;
        ang_accel_if.z = 0;
    }

    //---- Set Values ----
    rocket_.set_r_vect(pos_if);
    rocket_.set_r_dot(vel_if);
    rocket_.set_r_ddot(accel_if);
    rocket_.set_w_vect(ang_vel_if);
    rocket_.set_w_dot(ang_accel_if);
    rocket_.set_f_net(net_force_if);
    rocket_.set_t_net(net_torque_if);
    rocket_.set_q_ornt(orient);

};

Vector3 RungeKutta::calc_net_force(double tStamp, Vector3 vel_if) {
    
    /*************** Retrieve Instantaneous Rocket Parameters *****************/

    Vector3 thrust_rf = motor_.get_thrust(tStamp);

    double mass = rocket_.get_mass();
    double area = rocket_.get_A_ref();
    double c_Na = rocket_.get_Cna();  // normal force coefficient derivative
    double drag_coef = rocket_.get_Cd();

    /******************** Calculate Angular Acceleration **********************/

    Vector3 aero_force_rf;

    if (vel_if.magnitude() > 0.01) {
        Vector3 vel_rf = rocket_.i2r(vel_if);
        Vector3 normal_force_rf;

        double alpha = acos(vel_rf.z / vel_rf.magnitude());  // angle between velocity vector and rocket axis
        double normal_coef = c_Na * alpha;
        
        // 1.225 is temporary density of air
        double normal_force_mag = 0.5 * normal_coef * vel_rf.magnitude2() * area * 1.225;
        normal_force_rf.x = (-vel_rf.x);
        normal_force_rf.y = (-vel_rf.y);
        normal_force_rf.z = 0;

        normal_force_rf.normalize();
        normal_force_rf = normal_force_rf * normal_force_mag;

        double drag_mag = 0.5 * drag_coef * vel_rf.magnitude2() * area * 1.225;
        Vector3 drag_rf(0, 0, -(drag_mag));

        aero_force_rf = normal_force_rf + drag_rf;
    } else {
        aero_force_rf.x = 0;
        aero_force_rf.y = 0;
        aero_force_rf.z = 0;
    }

    Vector3 net_force_if = rocket_.r2i(aero_force_rf + thrust_rf);
    net_force_if.z -= (9.81 * mass);

    return net_force_if;
};

Vector3 RungeKutta::calc_net_torque(double tStamp, Vector3 vel_if, Vector3 ang_vel_if) {
    
    /*************** Retrieve Instantaneous Rocket Parameters *****************/

    Vector3 Cp_vect_rf = rocket_.get_Cp_vect();
    Vector3 thrust_rf = motor_.get_thrust(tStamp);

    double inertia[9];         // moments of inertia
    rocket_.get_I(inertia);

    double mass = rocket_.get_mass();
    double area = rocket_.get_A_ref();
    double c_Na = rocket_.get_Cna();  // normal force coefficient derivative
    double drag_coef = rocket_.get_Cd();

    /********************* Calculate Axial Acceleration ***********************/

    Vector3 aero_force_rf;
    Vector3 aero_torque_rf;
    Vector3 aero_force_if;
    Vector3 aero_torque_if;
    Vector3 net_force_rf;
    Vector3 net_torque_rf;

    if (vel_if.magnitude() > 0.01) {
        Vector3 vel_rf = rocket_.i2r(vel_if);
        Vector3 normal_force_rf;

        double alpha = acos(vel_rf.z / vel_rf.magnitude());  // angle between velocity vector and rocket axis
        double normal_coef = c_Na * alpha;
        
        // 1.225 is temporary density of air
        double normal_force_mag = 0.5 * normal_coef * vel_rf.magnitude2() * area * 1.225;
        normal_force_rf.x = (-vel_rf.x);
        normal_force_rf.y = (-vel_rf.y);
        normal_force_rf.z = 0;

        normal_force_rf.normalize();
        normal_force_rf = normal_force_rf * normal_force_mag;

        double drag_mag = 0.5 * drag_coef * vel_rf.magnitude2() * area * 1.225;
        Vector3 drag_rf(0, 0, -(drag_mag));

        aero_force_rf = normal_force_rf + drag_rf;
        aero_torque_rf = Cp_vect_rf.cross(aero_force_rf);
    } else {
        aero_torque_rf.x = 0;
        aero_torque_rf.y = 0;
        aero_torque_rf.z = 0;
    }

    Vector3 net_torque_if = rocket_.r2i(aero_torque_rf);

    return net_torque_if;
};
