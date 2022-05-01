/**
 * @file        Propulsion.h
 * @authors     Ayberk Yaraneri
 *
 * @brief       Class definitions for rocket propulsion components
 *
 * The classes defined here represent rocket propulsion components. While these
 * objects will usually represent rocket motors, they can also be defined to
 * represent components like gas thrusters or other force enducing mechanisms.
 *
 */

#pragma once

#ifndef _PROPULSION_H_
#define _PROPULSION_H_

#include <Eigen/Dense>
#include <string>
#include <vector>

using Eigen::Vector3d;

/*****************************************************************************/
/* RocketMotor Base Class and Derivatives                                    */
/*****************************************************************************/

/** RocketMotor Base Class
 * This class is a pure virtual class that represents any kind of
 * primary propulsion component intended to apply significant impulse to the
 * rocket. Examples of this are solid/liquid/hybrid motors, or anything else
 * that's designed to actually make the rocket liftoff and fly.
 *
 * The class is a pure virtual class, meaning it only exists to define an
 * interface that all rocket motor implementations should abide by.
 */
class RocketMotor {
   public:
    void ignite(double tStamp);
    bool is_burning(double tStamp) const;

    // Should this be a virtual implemented function?
    double get_propellant_mass(double tStamp) const;

    virtual double current_thrust(double tStamp) const = 0;
    virtual Vector3d get_thrust_vector(double tStamp) const = 0;

   protected:
    bool ignition_ = false;
    double ignition_tStamp_{0.0};
    double max_burn_duration_{0.0};
    double initial_propellant_mass_{0.0};
};

/** ConstantThrustSolidMotor Derived Class
 * Models a solid rocket motor that produces a constant
 * magnitude of thrust during the entirety of its burn duration.
 *
 * Not a very accurate model of a real motor, but useful for quick & dirty
 * simulations when you're too lazy to configure a proper thrust curve.
 */
class ConstantThrustSolidMotor : public RocketMotor {
   public:
    ConstantThrustSolidMotor(double max_burn_duration, double thrust_value,
                             double initial_propellant_mass)
        : thrust_value_(thrust_value) {
        max_burn_duration_ = max_burn_duration;
        initial_propellant_mass_ = initial_propellant_mass;
    };

    double current_thrust(double tStamp) const override;
    Vector3d get_thrust_vector(double tStamp) const override;

   private:
    // Thrust force value the motor will generate, constant for this motor type.
    double thrust_value_;
};

/** ThrustCurveSolidMotor Derived Class
 * Models a solid rocket motor that produces a varying magnitude of thrust as it
 * burns. The thrust-vs-time relationship is provided in a thrust curve .csv
 * file, which is then parsed into an Eigen matrix for future lookups.
 *
 * The thrust getter functions perform simple linear interpolation based on the
 * current simulation timestamp to lookup how much thrust the motor should be
 * generating.
 */
class ThrustCurveSolidMotor : public RocketMotor {
   public:
    ThrustCurveSolidMotor(std::string filename, double initial_propellant_mass);

    double current_thrust(double tStamp) const override;
    Vector3d get_thrust_vector(double tStamp) const override;

   private:
    // Useful metadata variables
    int data_points_;

    // Thrust curve lookup table
    Eigen::MatrixXd thrust_table_;
};

#endif
