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

void RocketMotor::log_motor_state(double tStamp) {
    // clang-format off
    std::stringstream datalog_ss;

    Vector3d thrust_vector_rf = get_thrust_vector(tStamp);

    datalog_ss << "[DATA] " 
               << tStamp << ","
               << is_burning(tStamp) << ","
               << get_propellant_mass(tStamp) << ","
               << current_thrust(tStamp) << ","
               << thrust_vector_rf.x() << ","
               << thrust_vector_rf.y() << ","
               << thrust_vector_rf.z();

    motor_logger_->info(datalog_ss.str());

    // clang-format on
}

/*****************************************************************************/
/* ConstantThrustSolidMotor Member Functions                                 */
/*****************************************************************************/

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
 * @param tStamp Current simulation timestamp
 * @return Vector3d The motor's current thrust vector
 */
ThrustCurveSolidMotor::ThrustCurveSolidMotor(
    std::string filename, double initial_propellant_mass,
    spdlog_basic_sink_ptr silsim_sink) {
    motor_logger_ =
        std::make_shared<spdlog::logger>("ThrustCurveSolidMotor", silsim_sink);
    motor_logger_->info("[DATALOG_FORMAT] " + datalog_format_string);

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
