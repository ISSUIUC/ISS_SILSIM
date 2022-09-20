/**
 * @file        CpuStateContext.h
 * @authors     Nicholas Phillips
 *
 * @brief       ???
 *
 */

#ifndef SILSIM_CPUSTATECONTEXT_H
#define SILSIM_CPUSTATECONTEXT_H

#include <PWMServo.h>
#include <dataLog.h>

#include <functional>
#include <ostream>

#include "Flaps.h"
#include "Sensor.h"

struct CpuStateContext {
    // function to add executable thread to the cpu
    std::function<void(void*)> add_thread;

    double system_time;
    sensorDataStruct_t sensorData;

    FSM_State rocketState = STATE_INIT;

    SerialClass Serial;
    SerialClass Serial1;

    KX134 highGimu;
    LSM9DS1 lowGimu;
    SFE_UBLOX_GNSS gps;
    MS5611 barometer;

    PWMServo servo_cw;   // Servo that induces clockwise roll moment
    PWMServo servo_ccw;  // Servo that counterclockwisei roll moment

    SDClass SD;
    // Create a struct that holds pointers to all the important objects needed
    // by the threads
    pointers sensor_pointers;

    uint8_t mpu_data[71];

    Gyroscope* gyroscope_pointer;
    Accelerometer* accelerometer_pointer;
    Barometer* barometer_pointer;
    GPSSensor* gps_pointer;
    Thermometer* thermometer_pointer;
    Magnetometer* magnetometer_pointer;
    Flaps* flaps;
    std::ostream* telemetry_log;
};

#endif  // SILSIM_CPUSTATECONTEXT_H
