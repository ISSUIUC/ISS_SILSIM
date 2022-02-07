/**
 * @file        RASAeroImport.cpp
 * @authors     Ayberk Yaraneri
 *              Kyler Norr
 *              Harry Zhao
 *
 * @brief      A utility class used to parse aerodynamic coefficients and
 * parameters generated by the RASAero software. The class stores the aero
 * parameters into a lookup table and makes them readily available through
 * getter functions. These getters interpolate the data as necessary.
 *
 */

#include "RASAeroImport.h"

#include <rapidcsv.h>

#include <cmath>
#include <iostream>
#include <set>
#include <string>

#include "Eigen/src/Core/Matrix.h"

// #define RASAERO_DEBUG

/**
 * @brief RASAeroImport class constructor. Parses data into lookup table
 *
 * @oaram file_path The path where the RASAero .csv data is found
 *
 */
RASAeroImport::RASAeroImport(std::string file_path) {
    rapidcsv::Document csv(file_path);

    auto mach = csv.GetColumn<double>("Mach Number");
    auto alpha = csv.GetColumn<double>("Alpha (deg)");
    auto protuberance = csv.GetColumn<double>("Protuberance (%)");
    auto cd_poweroff = csv.GetColumn<double>("CD Power-Off");
    auto cd_poweron = csv.GetColumn<double>("CD Power-On");
    auto ca_poweroff = csv.GetColumn<double>("CA Power-Off");
    auto ca_poweron = csv.GetColumn<double>("CA Power-On");
    auto cn = csv.GetColumn<double>("CN Total");
    auto cp = csv.GetColumn<double>("CP Total");

    int n_data = mach.size();

    aero_table_.resize(n_data, 9);

    for (int i = 0; i < n_data; i++) {
        aero_table_(i, 0) = mach[i];
        aero_table_(i, 1) = alpha[i];
        aero_table_(i, 2) = protuberance[i];
        aero_table_(i, 3) = cd_poweroff[i];
        aero_table_(i, 4) = cd_poweron[i];
        aero_table_(i, 5) = ca_poweroff[i];
        aero_table_(i, 6) = ca_poweron[i];
        aero_table_(i, 7) = cn[i];
        aero_table_(i, 8) = cp[i];
    }

    set_mach_number_params();
    set_alpha_params();
    set_protuberance_params();

#ifdef RASAERO_DEBUG
    std::cout << std::endl;
    std::cout << "### [RASAeroImport ctor parsing metadata]:" << std::endl;
    std::cout << "mach_instances = " << mach_number_instances_ << std::endl;
    std::cout << "mach_fidelity = " << mach_number_fidelity_ << std::endl;
    std::cout << "alpha_instances = " << alpha_instances_ << std::endl;
    std::cout << "alpha_fidelity = " << alpha_fidelity_ << std::endl;
    std::cout << "protuberance_instances = " << protuberance_instances_
              << std::endl;
    std::cout << "protuberance_fidelity = " << protuberance_fidelity_
              << std::endl;
#endif
}

/**
 * @brief Set useful metadata values regarding mach number for future lookups
 *
 * Calculates and sets the following values:
 * - mach_number_instances_: The number of unique mach number values that exist
 *       in the lookup table
 * - mach_number_fidelity_: The distance between two mach number instances
 *
 */
void RASAeroImport::set_mach_number_params() {
    auto column = aero_table_.col(0);
    auto vec = std::vector<double>(column.begin(), column.end());
    sort(vec.begin(), vec.end());
    vec.erase(unique(vec.begin(), vec.end()), vec.end());
    mach_number_instances_ = vec.size();
    mach_number_fidelity_ = fabs(vec[0] - vec[1]);
}

/**
 * @brief Set useful metadata values regarding alpha for future lookups
 *
 * FYI alpha = angle-of-attack
 *
 * Calculates and sets the following values:
 * - alpha_instances_: The number of unique alpha values that exist in the
 * lookup table
 * - alpha_fidelity_: The distance between two alpha instances
 *
 */
void RASAeroImport::set_alpha_params() {
    auto column = aero_table_.col(1);
    auto vec = std::vector<double>(column.begin(), column.end());
    sort(vec.begin(), vec.end());
    vec.erase(unique(vec.begin(), vec.end()), vec.end());
    alpha_instances_ = vec.size();
    alpha_fidelity_ = fabs(vec[0] - vec[1]);
}

/**
 * @brief Set useful metadata values regarding protuberance for future lookups
 *
 * Calculates and sets the following values:
 * - protuberance_instances_: The number of unique protuberance values that
 * exist in the lookup table
 * - protuberance_fidelity_: The distance between two protuberance instances
 *
 */
void RASAeroImport::set_protuberance_params() {
    auto column = aero_table_.col(2);
    auto vec = std::vector<double>(column.begin(), column.end());
    sort(vec.begin(), vec.end());
    vec.erase(unique(vec.begin(), vec.end()), vec.end());
    protuberance_instances_ = vec.size();
    protuberance_fidelity_ = fabs(vec[0] - vec[1]);
}

/** clang-format off
 *
 * @brief Performs a lookup and interpolates data using the Bilinear
 * Interpolation method.
 *
 * This one's a doozy.
 *
 * The function first finds the closest mach number instance in the lookup table
 * to the mach number that was requested. We chose not to interpolate among mach
 * number instances because the lookup table is quite rich in mach number
 * fidelity anyway, so just choosing the closest mach instance induces little
 * error and simplifies the interpolation significantly.
 *
 * A two-dimensional interpolation is then performed among the alpha and
 * protuberance instances that are immediately above and below the passed
 * parameters. This is done using Bilinear Interpolation:
 * https://en.wikipedia.org/wiki/Bilinear_interpolation
 *
 * The nomenclature used in the above Wiki page is translated to this
 * implementation as follows:
 * f(Q11) = f(alpha_below, prot_below) = row_a
 * f(Q21) = f(alpha_above, prot_below) = row_b
 * f(Q12) = f(alpha_below, prot_above) = row_c
 * f(Q22) = f(alpha_above, prot_above) = row_d
 * f(x,y1) = row_q
 * f(x,y2) = row_r
 *
 * @param mach The rocket's current Mach number
 * @param alpha The rocket's current angle-of-attack [degrees]
 * @param protuberance The current amount of protuberance. [0.0 to 1.0]
 *
 * @return RASAeroCoefficients Struct instance containing rocket's aero
 * coefficients at its current state
 *
 * clang-format on
 */
RASAeroCoefficients RASAeroImport::get_aero_coefficients(double mach,
                                                         double alpha,
                                                         double protuberance) {
    // Sanitize input
    mach = std::clamp(mach, kSmallestMach, kLargestMach);
    alpha = std::clamp(alpha, kSmallestAlpha, kLargestAlpha);
    protuberance = std::clamp(protuberance, kSmallestProtub, kSmallestProtub);

    // Find closest mach number to passed mach value
    double mach_below =
        std::trunc(mach / mach_number_fidelity_) * mach_number_fidelity_;
    double closest_mach = mach_below;
    if ((mach - mach_below) >= (mach_number_fidelity_ / 2.0))
        closest_mach = mach_below + mach_number_fidelity_;

    // Find alpha values to interpolate among
    double alpha_below = std::trunc(alpha / alpha_fidelity_) * alpha_fidelity_;
    double alpha_above = alpha_below + alpha_fidelity_;

    // Find protub values to interpolate among
    double prot_below = std::trunc(protuberance / protuberance_fidelity_) *
                        protuberance_fidelity_;
    double prot_above = prot_below + protuberance_fidelity_;

    // Start index of chunk with the mach number we want
    int mach_start_index =
        (std::round(closest_mach / mach_number_fidelity_) - 1) *
        (alpha_instances_ * protuberance_instances_);

    // How many rows to go down from start of chunk to get a,b,c,d rows
    int row_a_offset = (std::round(alpha_below / alpha_fidelity_) *
                        (protuberance_instances_)) +
                       std::round(prot_below / protuberance_fidelity_);

    int row_b_offset = (std::round(alpha_above / alpha_fidelity_) *
                        (protuberance_instances_)) +
                       std::round(prot_below / protuberance_fidelity_);

    int row_c_offset = (std::round(alpha_below / alpha_fidelity_) *
                        (protuberance_instances_)) +
                       std::round(prot_above / protuberance_fidelity_);

    int row_d_offset = (std::round(alpha_above / alpha_fidelity_) *
                        (protuberance_instances_)) +
                       std::round(prot_above / protuberance_fidelity_);

    // Fetch rows a,b,c,d from the lookup table
    Eigen::RowVectorXd row_a = aero_table_.row(mach_start_index + row_a_offset);
    Eigen::RowVectorXd row_b = aero_table_.row(mach_start_index + row_b_offset);
    Eigen::RowVectorXd row_c = aero_table_.row(mach_start_index + row_c_offset);
    Eigen::RowVectorXd row_d = aero_table_.row(mach_start_index + row_d_offset);

    // Perform Bilinear Interpolation
    Eigen::RowVectorXd row_q =
        (row_a * ((alpha_above - alpha) / (alpha_above - alpha_below))) +
        (row_b * ((alpha - alpha_below) / (alpha_above - alpha_below)));

    Eigen::RowVectorXd row_r =
        (row_c * ((alpha_above - alpha) / (alpha_above - alpha_below))) +
        (row_d * ((alpha - alpha_below) / (alpha_above - alpha_below)));

    Eigen::RowVectorXd row_z =
        (row_q * ((prot_above - protuberance) / (prot_above - prot_below))) +
        (row_r * ((protuberance - prot_below) / (prot_above - prot_below)));

    // Contemplate the meaning of life
    RASAeroCoefficients result{row_z(3), row_z(4), row_z(5),
                               row_z(6), row_z(7), row_z(8)};

#ifdef RASAERO_DEBUG
    std::cout << std::endl;
    std::cout << "### [RASAeroImport get_aero_coefficients() debug above/below "
                 "finding]:"
              << std::endl;
    std::cout << "mach_below = " << mach_below << std::endl;
    std::cout << "closest_mach = " << closest_mach << std::endl;
    std::cout << "alpha_below = " << alpha_below << std::endl;
    std::cout << "alpha_above = " << alpha_above << std::endl;
    std::cout << "prot_below = " << prot_below << std::endl;
    std::cout << "prot_above = " << prot_above << std::endl;

    std::cout << std::endl;
    std::cout
        << "### [RASAeroImport get_aero_coefficients() debug index finding]:"
        << std::endl;
    std::cout << "mach_start_index = " << mach_start_index << std::endl;
    std::cout << "row_a_offset = " << row_a_offset << std::endl;
    std::cout << "row_b_offset = " << row_b_offset << std::endl;
    std::cout << "row_c_offset = " << row_c_offset << std::endl;
    std::cout << "row_d_offset = " << row_d_offset << std::endl;

    std::cout << std::endl;

    std::cout << "### [RASAeroImport get_aero_coefficients() debug result]:"
              << std::endl;
    // std::cout << "given alpha = " << alpha << std::endl;
    std::cout << row_z << std::endl;
#endif

    return result;
}
