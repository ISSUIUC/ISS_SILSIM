/**
 * @file        RASAeroImport.h
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

#ifndef _RASAERO_IMPORT_H_
#define _RASAERO_IMPORT_H_

#include <Eigen/Dense>
#include <string>
#include <vector>

struct RASAeroCoefficients {
    double cd_poweroff;
    double cd_poweron;
    double ca_poweroff;
    double ca_poweron;
    double cn_total;
    double cp_total;
    double cd_protuberance;
};

class RASAeroImport {
   public:
    RASAeroImport(std::string file_path);

    /**************** Retrieve Aero Parameters of Rocket **********************/
    RASAeroCoefficients get_aero_coefficients(double mach, double alpha,
                                              double protuberance_percent);

    double get_CD_poweroff(double mach, double alpha,
                           double protuberance_percent);
    double get_CD_poweron(double mach, double alpha,
                          double protuberance_percent);
    double get_CA_poweroff(double mach, double alpha,
                           double protuberance_percent);
    double get_CA_poweron(double mach, double alpha,
                          double protuberance_percent);
    double get_CN_total(double mach, double alpha, double protuberance_percent);
    double get_CP_total(double mach, double alpha, double protuberance_percent);
    double get_CD_protuberance(double mach, double alpha,
                               double protuberance_percent);

   private:
    /************************** Set Parameters ********************************/
    void set_mach_number_params();
    void set_alpha_params();
    void set_protuberance_params();

    // Some metadata useful for interpolation
    int mach_number_instances_;    // Number of mach numbers in the table
    double mach_number_fidelity_;  // Distance between mach numbers in table

    int alpha_instances_;    // Number of alpha values in the table
    double alpha_fidelity_;  // Distance between alpha values in table

    int protuberance_instances_;    // Number of protoberance values in table
    double protuberance_fidelity_;  // Distance between protub. values in table

    // Aero Parameter Lookup Table
    Eigen::MatrixXd aero_table_;
};

#endif
