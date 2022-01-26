//
// Created by 16182 on 10/17/2021.
//

#ifndef SILSIM_CPUSTATECONTEXT_H
#define SILSIM_CPUSTATECONTEXT_H

#include <vector>
#include <Arduino.h>
#include <ChRt.h>
#include <PWMServo.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#include <KX134-1211.h>       //High-G IMU Library
#include <SparkFunLSM9DS1.h>  //Low-G IMU Library
#include <acShared.h>
#include <dataLog.h>
#include <hybridShared.h>
#include <pins.h>
#include <rocketFSM.h>
#include <sensors.h>
#include <ServoControl.h>
#include "Sensor.h"


struct CpuStateContext {
    double system_time;
    sensorDataStruct_t sensorData;

    FSM_State rocketState = STATE_INIT;

    KX134 highGimu;
    LSM9DS1 lowGimu;
    SFE_UBLOX_GNSS gps;

    PWMServo servo_cw;   // Servo that induces clockwise roll moment
    PWMServo servo_ccw;  // Servo that counterclockwisei roll moment

    // Create a struct that holds pointers to all the important objects needed by
    // the threads
    pointers sensor_pointers;

    uint8_t mpu_data[71];

    Gyroscope* gyroscope_pointer;
    Accelerometer* accelerometer_pointer;
    Barometer* barometer_pointer;

};

#endif  // SILSIM_CPUSTATECONTEXT_H
