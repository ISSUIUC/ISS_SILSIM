/**
 * @file        RASAeroImport.cpp
 * @authors     Ayberk Yaraneri
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
#include <string>
#include <iostream>
#include <set>

RASAeroImport::RASAeroImport(std::string file_path) {

    rapidcsv::Document csv(file_path);

    auto mach = csv.GetColumn<double>("Mach");
    auto alpha = csv.GetColumn<double>("Alpha");
    auto protuberance = csv.GetColumn<double>("Protuberance");
    auto cd = csv.GetColumn<double>("CD");
    auto ca_poweroff = csv.GetColumn<double>("CA Power-Off");
    auto ca_poweron = csv.GetColumn<double>("CA Power-On");
    auto cn = csv.GetColumn<double>("CN");
    auto cp = csv.GetColumn<double>("CP");
    auto cd_protuberance = csv.GetColumn<double>("Protuberance Drag");
    
    int n_data = mach.size();

    /*
    std::cout << "Number of Mach points = " << mach.size() << std::endl;
    std::cout << "Number of Alpha points = " << alpha.size() << std::endl;
    std::cout << "Number of protub points = " << protuberance.size() << std::endl;
    std::cout << "Number of cd points = " << cd.size() << std::endl;
    */

    aero_table_.resize(n_data, 9);

    for (int i = 0; i < n_data; i++) {
       aero_table_(i, 0) = mach[i];
       aero_table_(i, 1) = alpha[i];
       aero_table_(i, 2) = protuberance[i];
       aero_table_(i, 3) = cd[i];
       aero_table_(i, 4) = ca_poweroff[i];
       aero_table_(i, 5) = ca_poweron[i];
       aero_table_(i, 6) = cn[i];
       aero_table_(i, 7) = cp[i];
       aero_table_(i, 8) = cd_protuberance[i];
    }


    std::cout << aero_table_ << std::endl;

}

void RASAeroImport::set_mach_number_instances() {
    auto col = aero_table_.col(0);
    mach_number_instances_ = std::set<double> (col.begin(), col.end()).size();
}

void RASAeroImport::set_alpha_instances() {
    auto col = aero_table_.col(1);
    alpha_instances_ = std::set<double> (col.begin(), col.end()).size();
}

void RASAeroImport::set_protuberance_instances() {
    auto col = aero_table_.col(2);
    protuberance_instances_ = std::set<double> (col.begin(), col.end()).size();
}

void RASAeroImport::set_mach_number_fidelity() {
    mach_number_fidelity_ = fabs(aero_table_(0, 0) - aero_table_(1, 0));
}

void RASAeroImport::set_alpha_fidelity() {
    alpha_fidelity_ = fabs(aero_table_(0, 1) - aero_table_(1, 1));
}

void RASAeroImport::set_protuberance_fidelity() {
    protuberance_fidelity_ = fabs(aero_table_(0, 2) - aero_table_(1, 2));
}
