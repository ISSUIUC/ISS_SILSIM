//
// Created by 16182 on 10/3/2021.
//

#ifndef SILSIM_SPARKFUNLSM9DS1_H
#define SILSIM_SPARKFUNLSM9DS1_H

#include "LSM9DS1_Types.h"

#define LSM9DS1_AG_ADDR(i) 0
#define LSM9DS1_M_ADDR(i) 0

enum lsm9ds1_axis {
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    ALL_AXIS
};

class LSM9DS1
    {
       public:
        IMUSettings settings;

        // We'll store the gyro, accel, and magnetometer readings in a series of
        // public class variables. Each sensor gets three variables -- one for each
        // axis. Call readGyro(), readAccel(), and readMag() first, before using
        // these variables!
        // These values are the RAW signed 16-bit readings from the sensors.
        int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
        int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
        int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
        int16_t temperature; // Chip temperature
        float gRes, aRes, mRes; //Resolution Values
        float gBias[3], aBias[3], mBias[3];
        int16_t gBiasRaw[3], aBiasRaw[3], mBiasRaw[3];

        // LSM9DS1 -- LSM9DS1 class constructor
        // The constructor will set up a handful of private variables, and set the
        // communication mode as well.
        // Input:
        //	- interface = Either IMU_MODE_SPI or IMU_MODE_I2C, whichever you're using
        //				to talk to the IC.
        //	- xgAddr = If IMU_MODE_I2C, this is the I2C address of the accel/gyroscope.
        // 				If IMU_MODE_SPI, this is the chip select pin of the gyro (CS_AG)
        //	- mAddr = If IMU_MODE_I2C, this is the I2C address of the magnetometer.
        //				If IMU_MODE_SPI, this is the cs pin of the magnetometer (CS_M)
        LSM9DS1();

        // begin() and beginSPI() -- Initialize the gyro, accelerometer, and magnetometer.
        // This will set up the scale and output rate of each sensor. The values set
        // in the IMUSettings struct will take effect after calling this function.
        // INPUTS:
        // - agAddress - Sets either the I2C address of the accel/gyro or SPI chip
        //   select pin connected to the CS_XG pin.
        // - mAddress - Sets either the I2C address of the magnetometer or SPI chip
        //   select pin connected to the CS_M pin.
        // - i2C port (Note, only on "begin()" funtion, for use with I2C com interface)
        //   defaults to Wire, but if hardware supports it, can use other TwoWire ports.
        //   **For SPI use "beginSPI()", and only send first two address arguments.
//        uint16_t begin(uint8_t agAddress = LSM9DS1_AG_ADDR(1), uint8_t mAddress = LSM9DS1_M_ADDR(1), TwoWire &wirePort = Wire); //By default use the default I2C addres, and use Wire port
        uint16_t begin(uint8_t agAddress, uint8_t mAddress, TwoWire &wirePort); //By default use the default I2C addres, and use Wire port
        uint16_t beginSPI(uint8_t ag_CS_pin, uint8_t m_CS_pin);

        void calibrate(bool autoCalc = true);
        void calibrateMag(bool loadIn = true);
        void magOffset(uint8_t axis, int16_t offset);

        // accelAvailable() -- Polls the accelerometer status register to check
        // if new data is available.
        // Output:	1 - New data available
        //			0 - No new data available
        uint8_t accelAvailable();

        // gyroAvailable() -- Polls the gyroscope status register to check
        // if new data is available.
        // Output:	1 - New data available
        //			0 - No new data available
        uint8_t gyroAvailable();

        // tempAvailable() -- Polls the temperature status register to check
        // if new data is available.
        // Output:	1 - New data available
        //			0 - No new data available
        uint8_t tempAvailable();

        // magAvailable() -- Polls the accelerometer status register to check
        // if new data is available.
        // Input:
        //	- axis can be either X_AXIS, Y_AXIS, Z_AXIS, to check for new data
        //	  on one specific axis. Or ALL_AXIS (default) to check for new data
        //	  on all axes.
        // Output:	1 - New data available
        //			0 - No new data available
        uint8_t magAvailable(lsm9ds1_axis axis = ALL_AXIS);

        // readGyro() -- Read the gyroscope output registers.
        // This function will read all six gyroscope output registers.
        // The readings are stored in the class' gx, gy, and gz variables. Read
        // those _after_ calling readGyro().
        void readGyro();

        // int16_t readGyro(axis) -- Read a specific axis of the gyroscope.
        // [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
        // Input:
        //	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
        // Output:
        //	A 16-bit signed integer with sensor data on requested axis.
        int16_t readGyro(lsm9ds1_axis axis);

        // readAccel() -- Read the accelerometer output registers.
        // This function will read all six accelerometer output registers.
        // The readings are stored in the class' ax, ay, and az variables. Read
        // those _after_ calling readAccel().
        void readAccel();

        // int16_t readAccel(axis) -- Read a specific axis of the accelerometer.
        // [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
        // Input:
        //	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
        // Output:
        //	A 16-bit signed integer with sensor data on requested axis.
        int16_t readAccel(lsm9ds1_axis axis);

        // readMag() -- Read the magnetometer output registers.
        // This function will read all six magnetometer output registers.
        // The readings are stored in the class' mx, my, and mz variables. Read
        // those _after_ calling readMag().
        void readMag();

        // int16_t readMag(axis) -- Read a specific axis of the magnetometer.
        // [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
        // Input:
        //	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
        // Output:
        //	A 16-bit signed integer with sensor data on requested axis.
        int16_t readMag(lsm9ds1_axis axis);

        // readTemp() -- Read the temperature output register.
        // This function will read two temperature output registers.
        // The combined readings are stored in the class' temperature variables. Read
        // those _after_ calling readTemp().
        void readTemp();

        // calcGyro() -- Convert from RAW signed 16-bit value to degrees per second
        // This function reads in a signed 16-bit value and returns the scaled
        // DPS. This function relies on gScale and gRes being correct.
        // Input:
        //	- gyro = A signed 16-bit raw reading from the gyroscope.
        float calcGyro(int16_t gyro);

        // calcAccel() -- Convert from RAW signed 16-bit value to gravity (g's).
        // This function reads in a signed 16-bit value and returns the scaled
        // g's. This function relies on aScale and aRes being correct.
        // Input:
        //	- accel = A signed 16-bit raw reading from the accelerometer.
        float calcAccel(int16_t accel);

        // calcMag() -- Convert from RAW signed 16-bit value to Gauss (Gs)
        // This function reads in a signed 16-bit value and returns the scaled
        // Gs. This function relies on mScale and mRes being correct.
        // Input:
        //	- mag = A signed 16-bit raw reading from the magnetometer.
        float calcMag(int16_t mag);

        // setGyroScale() -- Set the full-scale range of the gyroscope.
        // This function can be called to set the scale of the gyroscope to
        // 245, 500, or 200 degrees per second.
        // Input:
        // 	- gScl = The desired gyroscope scale. Must be one of three possible
        //		values from the gyro_scale.
        void setGyroScale(uint16_t gScl);

        // setAccelScale() -- Set the full-scale range of the accelerometer.
        // This function can be called to set the scale of the accelerometer to
        // 2, 4, 6, 8, or 16 g's.
        // Input:
        // 	- aScl = The desired accelerometer scale. Must be one of five possible
        //		values from the accel_scale.
        void setAccelScale(uint8_t aScl);

        void setMagScale(uint8_t mScl);
    };

#endif  // SILSIM_SPARKFUNLSM9DS1_H
