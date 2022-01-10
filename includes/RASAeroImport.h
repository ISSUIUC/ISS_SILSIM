/**
 * @file        RASAeroImport.h
 * @authors     Ayberk Yaraneri
 *
 * @brief       
 *
 *
 */

#ifndef _RASAERO_IMPORT_H_
#define _RASAERO_IMPORT_H_

#include <string>

class RASAeroImport {
  public:

    RASAeroImport(std::string filePath);

    /**************** Retrieve Aero Parameters of Rocket **********************/
    double get_Cp_location(double mach, double alpha);
    double get_CNa(double mach, double alpha);    
    double get_CA(double mach, double alpha);

  private:


    /******************* Aero Parameter Lookup Table **************************/


};


#endif
