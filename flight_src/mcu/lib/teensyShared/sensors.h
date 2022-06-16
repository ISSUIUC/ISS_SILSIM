/**
 * @file        sensors.h
 * @authors     Jake Hawkins
 *              Anshuk Chigullapalli
 *              Ayberk Yaraneri
 *
 * @brief       Sensor (Low-G, High-G and GPS) struct definitions
 *
 * Here contains the pointer structs for three types of sensors (Low-g, High-g
 * and GPS) and the functions to acquire data from the sensors to the pointer
 * structs.
 *
 */

#ifndef SENSORS_H
#define SENSORS_H

#include "dataLog.h"

void lowGimuTickFunction(LSM9DS1* lsm_, DataLogBuffer* data_log_buffer,
                         LowGData* lowG_data);

void highGimuTickFunction(KX134* highG_, DataLogBuffer* data_log_buffer,
                          HighGData* highG_data);

void gpsTickFunction(SFE_UBLOX_GNSS* gps_, DataLogBuffer* data_log_buffer,
                     GpsData* gps_data);

void barometerTickFunction(MS5611* barometer_, DataLogBuffer* data_log_buffer,
                           BarometerData* barometer_data);

#endif