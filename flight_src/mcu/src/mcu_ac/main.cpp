/* main.cpp
 *   ______  ___     ___    ____
 *  /_  __/ / _ |   / _ \  / __/
 *   / /   / __ |  / , _/ _\ \
 *  /_/   /_/ |_| /_/|_| /___/
 *
 * Active Control Program
 *
 * Illinois Space Society - Avioinics Team
 *
 * Anshuk Chigullapalli
 * Josh Blustein
 * Ayberk Yaraneri
 * David Robbins
 * Matt Taylor
 * Ben Olaivar
 * Colin Kinsey
 * Grace Robbins
 */

#include <Arduino.h>
#include <ChRt.h>
#include <PWMServo.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#include "ActiveControl.h"
#include "KX134-1211.h"  //High-G IMU Library
#include "MS5611.h"      //Barometer library
#include "ServoControl.h"
#include "SparkFunLSM9DS1.h"                       //Low-G IMU Library
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"  //GPS Library
#include "acShared.h"
#include "dataLog.h"
#include "hybridShared.h"
#include "pins.h"
#include "rocketFSM.h"
#include "sensors.h"
#include "kalmanFilter.h"
#include "MadgwickAHRS.h"

// emulation for global variables
#include <CpuThread.h>

#include "GlobalVars.h"
#include <iostream>

using std::array;
#include<string>
using std::string;

// datalogger_THD datalogger_THD_vars;

//#define THREAD_DEBUG
//#define LOWGIMU_DEBUG
//#define HIGHGIMU_DEBUG
//#define GPS_DEBUG
//#define SERVO_DEBUG

/******************************************************************************/
/* ROCKET FINITE STATE MACHINE THREAD                                         */

class rocket_FSM : public CpuThread {
   public:
    rocket_FSM(void *arg, uint8_t prio)
        : CpuThread(prio),
          pointer_struct((struct pointers *)arg),
          stateMachine(pointer_struct) {}
    double loop(double timestamp) override {
#ifdef THREAD_DEBUG
        Serial.println("### Rocket FSM thread entrance");
#endif
        stateMachine.tickFSM();
        stateMachine.log_FSM_state(timestamp);
        return 10.0;  // FSM runs at 100 Hz
    }

   private:
    pointers *pointer_struct;
    rocketFSM stateMachine;
};

/******************************************************************************/
/* LOW G IMU THREAD                                                           */

class lowgIMU_THD : public CpuThread {
   public:
    lowgIMU_THD(void *arg, uint8_t prio)
        : CpuThread(prio), pointer_struct((struct pointers *)arg) {}
    double loop(double timestamp) override {
#ifdef THREAD_DEBUG
        Serial.println("### Low G IMU thread entrance");
#endif
        LSM9DS1 *lsm = pointer_struct->lowGimuPointer;
        DataLogBuffer *data_log_buffer =
            &pointer_struct->dataloggerTHDVarsPointer;
        LowGData *lowG_Data = &pointer_struct->sensorDataPointer->lowG_data;
        lowGimuTickFunction(lsm, data_log_buffer, lowG_Data);

        return 1.0;
    }

   private:
    struct pointers *pointer_struct;
};

/******************************************************************************/
/* BAROMETER THREAD                                                           */

class barometer_THD : public CpuThread {
   public:
    barometer_THD(void *arg, uint8_t prio)
        : CpuThread(prio), pointer_struct((struct pointers *)arg) {}
    // Load outside variables into the function

    double loop(double timestamp) override {
#ifdef THREAD_DEBUG
        Serial.println("### Barometer thread entrance");
#endif
        MS5611 * barometer_ = pointer_struct->barometerPointer;
        DataLogBuffer *data_log_buffer =
            &pointer_struct->dataloggerTHDVarsPointer;
        BarometerData *barometer_data =
            &pointer_struct->sensorDataPointer->barometer_data;
        barometerTickFunction(barometer_, data_log_buffer, barometer_data);

        return 6.0;
    }

   private:
    struct pointers *pointer_struct;
};

/******************************************************************************/
/* HIGH G IMU THREAD                                                          */

class highgIMU_THD : public CpuThread {
   public:
    highgIMU_THD(void *arg, uint8_t prio)
        : CpuThread(prio), pointer_struct((struct pointers *)arg) {}

    double loop(double timestamp) override {
#ifdef THREAD_DEBUG
        Serial.println("### High G IMU thread entrance");
#endif
        KX134 *highG = pointer_struct->highGimuPointer;
        DataLogBuffer *data_log_buffer =
            &pointer_struct->dataloggerTHDVarsPointer;
        HighGData *highg_data = &pointer_struct->sensorDataPointer->highG_data;

        highGimuTickFunction(highG, data_log_buffer, highg_data);

        return 6.0;
    }

   private:
    struct pointers *pointer_struct;
};

/******************************************************************************/
/* GPS THREAD                                                                 */
class gps_THD : public CpuThread {
   public:
    gps_THD(void *arg, uint8_t prio)
        : CpuThread(prio), pointer_struct((struct pointers *)arg) {}

    double loop(double timestamp) override {
#ifdef THREAD_DEBUG
        Serial.println("### GPS thread entrance");
#endif
        SFE_UBLOX_GNSS *gps_ = pointer_struct->GPSPointer;
        DataLogBuffer *data_log_buffer =
            &pointer_struct->dataloggerTHDVarsPointer;
        GpsData *gps_data = &pointer_struct->sensorDataPointer->gps_data;
        gpsTickFunction(gps_, data_log_buffer, gps_data);

#ifdef THREAD_DEBUG
        Serial.println("### GPS thread exit");
#endif

        return 80.0;
    }

   private:
    struct pointers *pointer_struct;
};

/******************************************************************************/
/* KALMAN FILTER THREAD                                                       */

class Kalman_Filter_THD : public CpuThread {
   public:
    Kalman_Filter_THD(void *arg, uint8_t prio)
        : CpuThread(prio),
          pointer_struct((struct pointers *)arg),
          Kf((struct pointers *)arg) {
        Serial.println("Initialize Kalman");
        Kf.Initialize();
    }

   double loop(double timestamp) override {
        Kf.kfTickFunction();

        Kf.log_kf_state(timestamp);

        //Serial.println("Predicted Alt:");
        // Serial.println(std::to_string(Kf.x_k(0, 0)).c_str());
        // Serial.println("Predicted Vel:");
        // Serial.println(std::to_string(Kf.x_k(1,0)).c_str());
        
        // chMtxLock(Kf.mutex_highG_);
        // Serial.println(std::to_string(*Kf.gz_H).c_str());
        // chMtxUnlock(Kf.mutex_highG_);
#ifdef THREAD_DEBUG
        Serial.println("### Kalman_Filter thread entrance");
#endif
        return 5.0;
    }

   private:
    KalmanFilter Kf;
    struct pointers *pointer_struct;
};

/******************************************************************************/
/* SERVO CONTROL THREAD                                                       */

class servo_THD : public CpuThread {
   public:
    servo_THD(void *arg, uint8_t prio)
        : CpuThread(prio),
          pointer_struct((struct pointers *)arg),
          ac(pointer_struct, &servo_cw) {
            ac.setLaunchPadElevation();
          }

    double loop(double timestamp) override {
#ifdef THREAD_DEBUG
        Serial.println("### Servo thread entrance");
#endif

        ac.ctrlTickFunction();
        ac.log_controller_state(timestamp);
        return 6.0;  // FSM runs at 100 Hz
    }

   private:
    pointers *pointer_struct;
    Controller ac;
};

/******************************************************************************/
/* MPU COMMUNICATION THREAD                                                   */

class mpuComm_THD : public CpuThread {
   public:
    mpuComm_THD(void *arg, uint8_t prio) : CpuThread(prio) {
        Serial1.begin(115200);
    }
    double loop(double timestamp) override {
#ifdef THREAD_DEBUG
        Serial.println("### mpuComm thread entrance");
#endif

        //! locking data from sensorData struct
        chMtxLock(&sensor_pointers.dataloggerTHDVarsPointer.dataMutex_lowG);

        digitalWrite(LED_WHITE, HIGH);

        // write transmission code here
        unsigned i = 3;  // because the first 3 indices are already set to be
        // ISS

        uint8_t *data =
            (uint8_t *)&sensor_pointers.sensorDataPointer->lowG_data;

        //! Unlocking &dataMutex
        chMtxUnlock(&sensor_pointers.dataloggerTHDVarsPointer.dataMutex_lowG);

        mpu_data[0] = 0x49;
        mpu_data[1] = 0x53;
        mpu_data[2] = 0x53;

        for (; i < 3 + sizeof(data); i++) {
            mpu_data[i] = *data;  // de-references to match data types, not sure
            // if correct, might send only the first byte
            data++;
        }

        // TODO: Send rocket state too? Is there a mutex for rocket state?

        Serial1.write(mpu_data, sizeof(mpu_data));

        digitalWrite(LED_WHITE, LOW);

        /* for (uint8_t i = 0; i < sizeof(mpu_data); ++i) {
                      Serial.printf("0x%.2X\t", mpu_data[i]);
              }
              Serial.printf("\n\n"); */

        return 6.0;
    }
};

/******************************************************************************/
/* DATA LOGGER THREAD                                                   */

class dataLogger_THD : public CpuThread {
   public:
    dataLogger_THD(void *arg, uint8_t prio)
        : CpuThread(prio), pointer_struct((struct pointers *)arg) {}

    double loop(double timestamp) override {
#ifdef THREAD_DEBUG
        Serial.println("Data Logging thread entrance");
#endif

        dataLoggerTickFunction(pointer_struct);

        return 6.0;
    }

   private:
    pointers *pointer_struct;
};

/**
 * @brief Starts all of the threads.
 *
 */
void chSetup() {
    // added play_THD for creation
    
    chThdCreateStatic(rocket_FSM_WA, sizeof(rocket_FSM_WA), NORMALPRIO,
                      rocket_FSM, &sensor_pointers);
    chThdCreateStatic(gps_WA, sizeof(gps_WA), NORMALPRIO, gps_THD,
                      &sensor_pointers);
    chThdCreateStatic(barometer_WA, sizeof(barometer_WA), NORMALPRIO + 1,
                      barometer_THD, &sensor_pointers);
    chThdCreateStatic(lowgIMU_WA, sizeof(lowgIMU_WA), NORMALPRIO, lowgIMU_THD,
                      &sensor_pointers);
    chThdCreateStatic(highgIMU_WA, sizeof(highgIMU_WA), NORMALPRIO,
                      highgIMU_THD, &sensor_pointers);
    chThdCreateStatic(servo_WA, sizeof(servo_WA), NORMALPRIO, servo_THD,
                      &sensor_pointers);
    chThdCreateStatic(lowg_dataLogger_WA, sizeof(lowg_dataLogger_WA),
                      NORMALPRIO, dataLogger_THD, &sensor_pointers);
    chThdCreateStatic(mpuComm_WA, sizeof(mpuComm_WA), NORMALPRIO, mpuComm_THD,
                      NULL);
    chThdCreateStatic(Kalman_Filter_WA, sizeof(Kalman_Filter_WA), NORMALPRIO, Kalman_Filter_THD, 
                      &sensor_pointers);
    Serial.println("Finish Setup");
}

void emu_setup() {
    int32_t temperature;

#if defined(THREAD_DEBUG) || defined(LOWGIMU_DEBUG) ||     \
    defined(BAROMETER_DEBUG) || defined(HIGHGIMU_DEBUG) || \
    defined(GPS_DEBUG) || defined(SERVO_DEBUG)
    Serial.begin(115200);
    while (!Serial) {
    }
#endif
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);
    pinMode(LED_WHITE, OUTPUT);

    digitalWrite(LED_BLUE, HIGH);
    digitalWrite(LED_ORANGE, HIGH);

    // TODO: Don't forget this
    Serial.println("------------------------------------------------");

    sensor_pointers.lowGimuPointer = &lowGimu;
    sensor_pointers.highGimuPointer = &highGimu;
    sensor_pointers.barometerPointer = &barometer;
    sensor_pointers.GPSPointer = &gps;
    sensor_pointers.sensorDataPointer = &sensorData;

    SPI.begin();

    // Initialize barometer
    barometer.init();

    // lowGimu setup
    if (lowGimu.beginSPI(LSM9DS1_AG_CS, LSM9DS1_M_CS) ==
        false)  // note, we need to sent this our CS pins (defined above)
    {
        digitalWrite(LED_RED, HIGH);
        Serial.println("Failed to communicate with LSM9DS1. Stalling Program");
        return;
    }

    lowGimu.setAccelScale(16);
    lowGimu.setGyroScale(2000);
    lowGimu.setMagScale(16);

    // GPS Setup
    if (!gps.begin(SPI, ZOEM8Q0_CS, 4000000)) {
        digitalWrite(LED_RED, HIGH);
        Serial.println(
            "Failed to communicate with ZOEM8Q0 gps. Stalling Program");
        return;
    }
    gps.setPortOutput(COM_PORT_SPI,
                      COM_TYPE_UBX);  // Set the SPI port to output UBX only
    // (turn off NMEA noise)
    gps.saveConfigSelective(
        VAL_CFG_SUBSEC_IOPORT);  // Save (only) the communications port settings
    // to flash and BBR
    gps.setNavigationFrequency(10);  // set sampling rate to 10hz

    // SD Card Setup
    if (SD.begin(BUILTIN_SDCARD)) {
        char file_extension[6] = ".dat";

        char data_name[16] = "data";
        // Initialize SD card
        sensor_pointers.dataloggerTHDVarsPointer.dataFile =
            SD.open(sd_file_namer(data_name, file_extension),
                    O_CREAT | O_WRITE | O_TRUNC);
        // print header to file on sd card that lists each variable that is
        // logged
        sensor_pointers.dataloggerTHDVarsPointer.dataFile.println(
            "ax,ay,az,gx,gy,gz,mx,my,mz,ts_lowg,"
            "hg_ax,hg_ay,hg_az,ts_highg,"
            "latitude,longitude,altitude,GPS Lock,ts_gps,"
            "state_q0,state_q1,state_q2,state_q3,state_x,state_y,state_z,state_"
            "vx,state_vy,state_vz,"
            "state_ax,state_ay,state_az,state_omegax,state_omegay,state_omegaz,"
            "state_latitude,state_longitude,ts_state,"
            "rocketState,ts_RS");
        sensor_pointers.dataloggerTHDVarsPointer.dataFile.flush();
        // Serial.println(lowg_datalogger_THD_vars.dataFile.name());
    } else {
        digitalWrite(LED_RED, HIGH);
        Serial.println("SD Begin Failed. Stalling Program");
        return;
    }

    // Servo Setup
    servo_cw.attach(SERVO_CW_PIN, 770, 2250);


    Serial.println("Starting ChibiOS");
    chBegin(chSetup);
}
