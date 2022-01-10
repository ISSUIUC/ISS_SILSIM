//
// Created by 16182 on 10/17/2021.
//

#ifndef SILSIM_CPUSTATECONTEXT_H
#define SILSIM_CPUSTATECONTEXT_H


#include <Arduino.h>
#include <ChRt.h>
#include <PWMServo.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#include <KX134-1211.h>       //High-G IMU Library
#include <SparkFunLSM9DS1.h>  //Low-G IMU Library
#include <ZOEM8Q0.hpp>        //GPS Library
#include <acShared.h>
#include <dataLog.h>
#include <hybridShared.h>
#include <pins.h>
#include <rocketFSM.h>
#include <sensors.h>
#include <ServoControl.h>

struct CpuStateContext {
    double system_time;
    sensorDataStruct_t sensorData;

    FSM_State rocketState = STATE_INIT;

    KX134 highGimu;
    LSM9DS1 lowGimu;
    ZOEM8Q0 gps = ZOEM8Q0();

    PWMServo servo_cw;   // Servo that induces clockwise roll moment
    PWMServo servo_ccw;  // Servo that counterclockwisei roll moment

    // Create a struct that holds pointers to all the important objects needed by
    // the threads
    pointers sensor_pointers;

    uint8_t mpu_data[71];
};

#endif  // SILSIM_CPUSTATECONTEXT_H
