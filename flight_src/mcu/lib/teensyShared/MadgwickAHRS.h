/**
 * @file        MadgwickAHRS.h
 * @authors     Ayberk Yaraneri
 *
 * @brief   A wrapper class that implements the Madwick AHRS filter for attitude estimation.
 *
 * This class simply wraps the functionality of the Madgwick AHRS filter in a C++ class.
 *
 * Implementation of Madgwick's IMU and AHRS algorithms:
 * http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 */

#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

#include "sensors.h"

#include "GlobalVars.h"

#define sampleFreq	100.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain

class MadgwickAHRS {
    public:
        MadgwickAHRS(pointers* ptr) {
            pointer_struct_ = ptr;

            state_data_ptr_ = &pointer_struct_->stateData;
            lowG_data_ptr_ = &pointer_struct_->sensorDataPointer->lowG_data;
            dataMutex_lowG_ = &pointer_struct_->dataloggerTHDVarsPointer.dataMutex_lowG;
            dataMutex_state_ = &pointer_struct_->dataloggerTHDVarsPointer.dataMutex_state;

            // SILSIM Data Logging
            madgwick_logger_ = std::make_shared<spdlog::logger>("MadgwickAHRS", silsim_datalog_sink);
            madgwick_logger_->info("DATALOG_FORMAT," + datalog_format_string);
        };

        void MadgwickAHRSupdate();
        void MadgwickAHRSupdateIMU();

        // SILSIM Data Logging
        void log_madgwick_state(double tStamp);

    private:
        pointers* pointer_struct_;
        StateData* state_data_ptr_;
        LowGData* lowG_data_ptr_;

        mutex_t* dataMutex_lowG_;
        mutex_t* dataMutex_state_;

        float beta_ = betaDef;
        float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;

        float invSqrt(float x);

        // SILSIM Data Logging
        std::shared_ptr<spdlog::logger> madgwick_logger_;
        std::string datalog_format_string = 
            "timestamp,q0,q1,q2,q3";
};

#endif // MADGWICK_AHRS_H
