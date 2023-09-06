/**
 * @file        Propulsion.cpp
 * @authors     Ayberk Yaraneri
 *
 * @brief       Member function implementations of propulsion component
 * classes
 *
 * The classes defined here represent rocket propulsion components. While these
 * objects will usually represent rocket motors, they can also be defined to
 * represent components like gas thrusters or other force enducing mechanisms.
 *
 */

#include "Propulsion.h"

#include <rapidcsv.h>

#include <Eigen/Dense>

using Eigen::Vector3d;

/*****************************************************************************/
/* RocketMotor Member Functions                                              */
/*****************************************************************************/

/**
 * @brief Transitions state of Motor to ignited (producing thrust)
 *
 * @param tStamp Current simulation timestamp
 */
void RocketMotor::ignite(double tStamp) {
    ignition_ = true;
    ignition_tStamp_ = tStamp;
}

/**
 * @brief Check if motor is currently burning and producing thrust
 *
 * @param tStamp Current simulation timestamp
 * @return bool True if motor is burning and producing thrust, false otherwise
 */
bool RocketMotor::is_burning(double tStamp) const {
    double burn_end_tStamp = ignition_tStamp_ + max_burn_duration_;
    return (ignition_ && (tStamp <= burn_end_tStamp));
}

/**
 * @brief Calculate the mass of propellant currently in motor
 *
 * Performs a simple linear interpolation between the initial motor propellant
 * mass and zero depending on how long motor has been burning
 *
 * @param tStamp Current simulation timestamp
 * @return double Current propellant mass within the motor
 */
double RocketMotor::get_propellant_mass(double tStamp) const {
    if (!ignition_) return initial_propellant_mass_;

    if ((tStamp - ignition_tStamp_) > max_burn_duration_) return 0.0;

    return initial_propellant_mass_ *
           (1.0 - ((tStamp - ignition_tStamp_) / max_burn_duration_));
}

/*****************************************************************************/
/* ConstantThrustSolidMotor Member Functions                                 */
/*****************************************************************************/

/**
 * @brief Constructor for the ConstantThrustSolidMotor class
 *
 * Class represents a solid rocket motor that produces thrust that doesn't
 * change for the duration of the burni. i.e. it follows a flat thrust curve
 *
 * @param max_burn_duration Motor burn duration in seconds
 * @param thrust_value Thrust force the motor produces in Newtons
 * @param silsim_sink A pointer to SILSIM's data datlog sink. Can be nullptr!
 */
ConstantThrustSolidMotor::ConstantThrustSolidMotor(
    double max_burn_duration, double thrust_value,
    double initial_propellant_mass)
    : thrust_value_(thrust_value) {
    max_burn_duration_ = max_burn_duration;
    initial_propellant_mass_ = initial_propellant_mass;
}

/**
 * @brief Get the magnitude of the thrust force the motor is generating
 *
 * @param tStamp Current simulation timestamp
 * @return double Current thrust force magnitude
 */
double ConstantThrustSolidMotor::current_thrust(double tStamp) const {
    if (ignition_ && ((tStamp - ignition_tStamp_) <= max_burn_duration_))
        return thrust_value_;

    return 0.0;
}

/**
 * @brief Thrust vector getter function (by value)
 *
 * @param tStamp Current simulation timestamp
 * @return Vector3d The motor's current thrust vector
 */
Vector3d ConstantThrustSolidMotor::get_thrust_vector(double tStamp) const {
    return {0.0, 0.0, current_thrust(tStamp)};
}

/*****************************************************************************/
/* ThrustCurveSolidMotor Member Functions                                    */
/*****************************************************************************/

/**
 * @brief Constructor for the ThrustCurveSolidMotor class
 *
 * Class represents a solid rocket motor that produces thrust defineid by a
 * thrust curve file. The class interpolates between data points provided in the
 * curve.
 *
 * @param filename Relative filepath string to thrust curve .csv file
 * @param initial_propellant_mass The total mass of propellant before ignition
 * @param silsim_sink A pointer to SILSIM's data datlog sink. Can be nullptr!
 */
ThrustCurveSolidMotor::ThrustCurveSolidMotor(std::string filename,
                                             double initial_propellant_mass) {
    rapidcsv::Document csv(filename);
    std::vector<double> time_vals = csv.GetColumn<double>("Time");
    std::vector<double> thrust_vals = csv.GetColumn<double>("Thrust");

    data_points_ = time_vals.size();
    thrust_table_.resize(data_points_, 2);

    for (int i = 0; i < data_points_; ++i) {
        thrust_table_(i, 0) = time_vals[i];
        thrust_table_(i, 1) = thrust_vals[i];
    }

    max_burn_duration_ = time_vals[data_points_ - 1];
    initial_propellant_mass_ = initial_propellant_mass;
}

/**
 * @brief Get the magnitude of the thrust force the motor is generating
 *
 * @param tStamp Current simulation timestamp
 * @return double Current thrust force magnitude
 */
double ThrustCurveSolidMotor::current_thrust(double tStamp) const {
    if (!ignition_ || ((tStamp - ignition_tStamp_) > max_burn_duration_) ||
        (tStamp < 0.0))
        return 0.0;

    for (int i = 1; i < data_points_; i++) {
        double burn_time = tStamp - ignition_tStamp_;
        if (burn_time < thrust_table_(i, 0)) {
            double slope = (thrust_table_(i, 1) - thrust_table_(i - 1, 1)) /
                           (thrust_table_(i, 0) - thrust_table_(i - 1, 0));
            double thrust = slope * (burn_time - thrust_table_(i - 1, 0)) +
                            thrust_table_(i - 1, 1);

            return thrust;
        }
    }

    return 0.0;
}

/**
 * @brief Thrust vector getter function (by value)
 *
 * @param tStamp Current simulation timestamp
 * @return Vector3d The motor's current thrust vector
 */
Vector3d ThrustCurveSolidMotor::get_thrust_vector(double tStamp) const {
    return {0.0, 0.0, current_thrust(tStamp)};
}
