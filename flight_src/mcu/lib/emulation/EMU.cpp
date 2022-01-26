//
// Created by 16182 on 10/5/2021.
//
#include "../../../../includes/CpuStateContext.h"
#include "Arduino.h"
#include "ChRt.h"
#include "KX134-1211.h"
#include "MS5611.h"
#include "PWMServo.h"
#include "SPI.h"
#include "SparkFunLSM9DS1.h"
#include "SD.h"


extern CpuStateContext context;
//context.system_time


void chMtxLock(mutex_t * mtx){
    mtx->lock();
}

void chMtxUnlock(mutex_t * mtx){
    mtx->unlock();
}

uint32_t chVTGetSystemTime(){
    return context.system_time / 1000;
}
void chThdSleepMilliseconds(uint32_t ms) {
    throw std::runtime_error("chThdSleepMilliseconds needs to be replaced to run within the simulator");
}
void chBegin(void (*mainThread)()) {
    mainThread();
}
void chSysUnlock() {}
void chSysLock() {}

void SerialClass::println(const char * str){
    puts(str);
}
void SerialClass::begin(uint32_t frequency) {}
void SerialClass::write(void *data, uint32_t size) {}

void digitalWrite(uint8_t pin, uint8_t val){}
void pinMode(uint8_t pin, uint8_t mode){}
void delay(uint32_t ms) {
    throw std::runtime_error("delay needs to be replaced to run within the simulator");
}



KX134::KX134() {}
void KX134::update_data() {
    Eigen::Vector3d data;
    context.accelerometer_pointer->get_data(data);

    Eigen::Vector3d scaled = data * 2048;
    x_accel = scaled.x();
    y_accel = scaled.y();
    z_accel = scaled.z();
}
int16_t KX134::binary_to_decimal(int16_t) { return 0; }
void KX134::init() {}
int16_t KX134::get_x_accel_raw() { return x_accel; }
int16_t KX134::get_y_accel_raw() { return y_accel; }
int16_t KX134::get_z_accel_raw() { return z_accel; }
float KX134::get_x_gforce() {
    int16_t decimal = get_x_accel_raw();
    float gForce = (float)decimal / 2048;
    return gForce;
}
float KX134::get_y_gforce() {
    int16_t decimal = get_y_accel_raw();
    float gForce = (float)decimal / 2048;
    return gForce;
}
float KX134::get_z_gforce() {
    int16_t decimal = get_z_accel_raw();
    float gForce = (float)decimal / 2048;
    return gForce;
}
float KX134::get_x_accel() {
    float gForce = get_x_gforce();
    return gForce * 9.8;
}
float KX134::get_y_accel() {
    float gForce = get_y_gforce();
    return gForce * 9.8;
}
float KX134::get_z_accel() {
    float gForce = get_z_gforce();
    return gForce * 9.8;
}

PWMServo::PWMServo() {}
uint8_t PWMServo::attach(int pinArg, int min, int max) { return 0; }
void PWMServo::write(int angleArg) {}
void PWMServo::detach() {}
uint8_t PWMServo::attached() { return 0; }



LSM9DS1::LSM9DS1() {}
uint16_t LSM9DS1::begin(uint8_t agAddress, uint8_t mAddress, TwoWire &wirePort) {return 0;}
uint16_t LSM9DS1::beginSPI(uint8_t ag_CS_pin, uint8_t m_CS_pin) { return 0; }
void LSM9DS1::calibrate(bool autoCalc) {}
void LSM9DS1::calibrateMag(bool loadIn) {}
void LSM9DS1::magOffset(uint8_t axis, int16_t offset) {}
void LSM9DS1::readGyro() {
    Eigen::Vector3d data;
    context.gyroscope_pointer->get_data(data);
    float gScale = 0.00875;

    Eigen::Vector3d scaled = data * gScale;
    gx = scaled.x();
    gy = scaled.y();
    gz = scaled.z();
}
void LSM9DS1::readAccel() {
    Eigen::Vector3d data;
    context.accelerometer_pointer->get_data(data);
    float aScale = 0.000061;

    Eigen::Vector3d scaled = data * aScale;
    ax = scaled.x();
    ay = scaled.y();
    az = scaled.z();
}
float LSM9DS1::calcGyro(int16_t gyro) {
    float gScale = 0.00875;
    return gyro * gScale;
}
float LSM9DS1::calcAccel(int16_t accel) {
    float aScale = 0.000061;
    return accel * aScale;
}
void LSM9DS1::setAccelScale(uint8_t aScl) {}
void LSM9DS1::readMag() {
    Eigen::Vector3d data;
    context.magnetometer_pointer->get_data(data);
    float mScale = 0.00014;

    Eigen::Vector3d scaled = data * mScale;
    mx = scaled.x();
    my = scaled.y();
    mz = scaled.z();
}
float LSM9DS1::calcMag(int16_t mag) {
    float mScale = 0.00014;
    return mag * mScale;
}

MS5611::MS5611(uint8_t pin){}
void MS5611::init(){}
int MS5611::read(uint8_t bits) {
    double tempKelvins = context.barometer_pointer->get_data(); //init data is in Kelvins
    double tempCelsius = tempKelvins - 273.15;                  //first convert to celsius
    _temperature = tempCelsius * 100;                           //finally convert celsius to hundreths of degrees celsius
    _pressure = context.barometer_pointer->get_data();
}
uint32_t MS5611::getPressure() const { return _pressure; } 
int32_t MS5611::getTemperature() const { return _temperature; }

SPIClass SPI{};
void SPIClass::begin() {}

bool SFE_UBLOX_GNSS::getPVT(uint16_t maxWait){
    Eigen::Vector3d data;
    context.gps_pointer->get_data(data);

    double BigDegrees = data.x / 111036.53;  //converted meters to 'rough' degrees @ 40.1164 degrees latitude
    double BigDegrees = data.y / 85269.13;  //converted meters to 'rough' degrees @ 40.1164 degrees latitude
    double FatHeight = data.z * 1000 ; //converted meters to millimeters
    _Longitude = BigDegrees * 10E+7;
    _Latitude = BigDegrees * 10E+7;
    _Altitude = FatHeight;

    _isFresh = true;
    _isFreshAltitude = true;
    _isFreshLatitude = true;
    _isFreshLongitude = true;
    return true;
}
int32_t SFE_UBLOX_GNSS::getLatitude(uint16_t maxWait){
    if(!_isFreshLatitude){
        getPVT(maxWait);
    }
    _isFreshLatitude = false;
    _isFresh = false;
    return _Latitude;
}
int32_t SFE_UBLOX_GNSS::getLongitude(uint16_t maxWait){
    if(!_isFreshLongitude){
        getPVT(maxWait);
    }
    _isFreshLongitude = false;
    _isFresh = false;
    return _Longitude;
}
int32_t SFE_UBLOX_GNSS::getAltitude(uint16_t maxWait){
    if(!_isFreshAltitude){
        getPVT(maxWait);
    }
    _isFreshAltitude = false;
    _isFresh = false;
    return _Altitude;
}
uint8_t SFE_UBLOX_GNSS::getFixType(uint16_t maxWait){return 0;}
uint8_t SFE_UBLOX_GNSS::getSIV(uint16_t maxWait){return 0;}
bool SFE_UBLOX_GNSS::begin(SPIClass &spiPort, uint8_t csPin, uint32_t spiSpeed){return false;}
bool SFE_UBLOX_GNSS::setPortOutput(uint8_t portID, uint8_t comSettings, uint16_t maxWait){return false;}
bool SFE_UBLOX_GNSS::saveConfigSelective(uint32_t configMask, uint16_t maxWait){return false;}
bool SFE_UBLOX_GNSS::setNavigationFrequency(uint8_t navFreq, uint16_t maxWait){return false;}



SDClass SD{};
bool SDClass::exists(const char *fileName) { return false; }
bool SDClass::begin(uint8_t csPin) { return false; }
File SDClass::open(const char *file_path, uint8_t mode) { return File(); }



void File::write(const uint8_t *data, size_t size) {}
void File::flush() {}
size_t File::println(const char *s) { return 0; }
