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

class RocketMotor {
   public:
    void ignite(double tStamp);
    bool is_burning(double tStamp) const;

    virtual double current_thrust(double tStamp) const = 0;
    virtual Vector3d get_thrust_vector(double tStamp) const = 0;

   protected:
    bool ignition_ = false;
    double ignition_tStamp_{0.0};
    double max_burn_duration_{0.0};
};

class ConstantThrustSolidMotor : public RocketMotor {
   public:
    ConstantThrustSolidMotor(double max_burn_duration, double thrust_value)
        : thrust_value_(thrust_value){ max_burn_duration_ = max_burn_duration; };

    double current_thrust(double tStamp) const override;
    Vector3d get_thrust_vector(double tStamp) const override;

   private:
    // Thrust force value the motor will generate, constant for this motor type.
    double thrust_value_;
};

class ThrustCurveSolidMotor : public RocketMotor {
   public:
    ThrustCurveSolidMotor(std::string filename);

    double current_thrust(double tStamp) const override;
    Vector3d get_thrust_vector(double tStamp) const override;

   private:
    // Useful metadata variables
    int data_points_;

    // Thrust curve lookup table
    Eigen::MatrixXd thrust_table_;
};

#endif
