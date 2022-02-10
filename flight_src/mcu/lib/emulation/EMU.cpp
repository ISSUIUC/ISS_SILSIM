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


//Define Table for resolutions (for aRes, gRes, mRes)
#define SENSITIVITY_ACCELEROMETER_2  0.000061
#define SENSITIVITY_ACCELEROMETER_4  0.000122
#define SENSITIVITY_ACCELEROMETER_8  0.000244
#define SENSITIVITY_ACCELEROMETER_16 0.000732
#define SENSITIVITY_GYROSCOPE_245    0.00875
#define SENSITIVITY_GYROSCOPE_500    0.0175
#define SENSITIVITY_GYROSCOPE_2000   0.07
#define SENSITIVITY_MAGNETOMETER_4   0.00014
#define SENSITIVITY_MAGNETOMETER_8   0.00029
#define SENSITIVITY_MAGNETOMETER_12  0.00043
#define SENSITIVITY_MAGNETOMETER_16  0.00058

extern CpuStateContext* global_context;
//context.system_time


void chMtxLock(mutex_t * mtx){
    mtx->lock();
}

void chMtxUnlock(mutex_t * mtx){
    mtx->unlock();
}

uint32_t chVTGetSystemTime(){
    return global_context->system_time * CH_CFG_ST_FREQUENCY;
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
bool SerialClass::operator!() { return false; }
SerialClass::operator bool() { return true; }

void digitalWrite(uint8_t pin, uint8_t val){}
void pinMode(uint8_t pin, uint8_t mode){}
void delay(uint32_t ms) {
    throw std::runtime_error("delay needs to be replaced to run within the simulator");
}



KX134::KX134() {}
void KX134::update_data() {
    Eigen::Vector3d data;
    global_context->accelerometer_pointer->get_data(data);

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
uint16_t LSM9DS1::begin(uint8_t agAddress, uint8_t mAddress, TwoWire &wirePort) {return 1;}
uint16_t LSM9DS1::beginSPI(uint8_t ag_CS_pin, uint8_t m_CS_pin) { return 1; }
void LSM9DS1::calibrate(bool autoCalc) {}
void LSM9DS1::calibrateMag(bool loadIn) {}
void LSM9DS1::magOffset(uint8_t axis, int16_t offset) {}
void LSM9DS1::readGyro() {
    Eigen::Vector3d data;
    global_context->gyroscope_pointer->get_data(data);
    float gScale = 0.00875;

    Eigen::Vector3d scaled = data / gScale;
    gx = scaled.x();
    gy = scaled.y();
    gz = scaled.z();
}
void LSM9DS1::readAccel() {
    Eigen::Vector3d data;
    global_context->accelerometer_pointer->get_data(data);

    Eigen::Vector3d scaled = data / aRes;
    ax = scaled.x();
    ay = scaled.y();
    az = scaled.z();
}
float LSM9DS1::calcGyro(int16_t gyro) {
    return gyro * gRes;
}
float LSM9DS1::calcAccel(int16_t accel) {
    return accel * aRes;
}
void LSM9DS1::setAccelScale(uint8_t aScl) {
    if (aScl == 2) {
        aRes = SENSITIVITY_ACCELEROMETER_2;
    } else if (aScl == 4) {
        aRes = SENSITIVITY_ACCELEROMETER_4;
    } else if (aScl == 8) {
        aRes = SENSITIVITY_ACCELEROMETER_8;
    } else if (aScl == 16) {
        aRes = SENSITIVITY_ACCELEROMETER_16;
    } else {
        aRes = SENSITIVITY_ACCELEROMETER_2;
    }
}

void LSM9DS1::setMagScale(uint8_t mScl) {
    if (mScl == 12) {
        mRes = SENSITIVITY_MAGNETOMETER_12;
    } else if (mScl == 4) {
        mRes = SENSITIVITY_MAGNETOMETER_4;
    } else if (mScl == 8) {
        mRes = SENSITIVITY_MAGNETOMETER_8;
    } else if (mScl == 16) {
        mRes = SENSITIVITY_MAGNETOMETER_16;
    } else {
        mRes = SENSITIVITY_MAGNETOMETER_4;
    }
}

void LSM9DS1::setGyroScale(uint16_t gScl) {
    if (gScl == 245) {
        gRes = SENSITIVITY_GYROSCOPE_245;
    } else if (gScl == 500) {
        gRes = SENSITIVITY_GYROSCOPE_500;
    } else if (gScl == 2000) {
        gRes = SENSITIVITY_GYROSCOPE_2000;
    } else {
        gRes = SENSITIVITY_GYROSCOPE_245;
    }
}

void LSM9DS1::readMag() {
    Eigen::Vector3d data;
    global_context->magnetometer_pointer->get_data(data);

    Eigen::Vector3d scaled = data / mRes;
    mx = scaled.x();
    my = scaled.y();
    mz = scaled.z();
}
float LSM9DS1::calcMag(int16_t mag) {
    return mag * mRes;
}

MS5611::MS5611(uint8_t pin){}
void MS5611::init(){}
int MS5611::read(uint8_t bits) {
    double tempKelvins = global_context->thermometer_pointer->get_data(); //init data is in Kelvins
    double tempCelsius = tempKelvins - 273.15;                  //first convert to celsius
    _temperature = tempCelsius * 100;                           //finally convert celsius to hundreths of degrees celsius
    _pressure = global_context->barometer_pointer->get_data();
    return 0;
}
uint32_t MS5611::getPressure() const { return _pressure; } 
int32_t MS5611::getTemperature() const { return _temperature; }
MS5611::MS5611() {}

SPIClass SPI{};
void SPIClass::begin() {}

bool SFE_UBLOX_GNSS::getPVT(uint16_t maxWait){
    Eigen::Vector3d data;
    global_context->gps_pointer->get_data(data);

    double BigLatDegrees = data.x() / 111036.53;  //converted meters to 'rough' degrees @ 40.1164 degrees latitude
    double BigLongDegrees = data.y() / 85269.13;  //converted meters to 'rough' degrees @ 40.1164 degrees latitude
    double FatHeight = data.z() * 1000 ; //converted meters to millimeters
    _Longitude = BigLongDegrees * 10E+7;
    _Latitude = BigLatDegrees * 10E+7;
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
bool SFE_UBLOX_GNSS::begin(SPIClass &spiPort, uint8_t csPin, uint32_t spiSpeed){return true;}
bool SFE_UBLOX_GNSS::setPortOutput(uint8_t portID, uint8_t comSettings, uint16_t maxWait){return false;}
bool SFE_UBLOX_GNSS::saveConfigSelective(uint32_t configMask, uint16_t maxWait){return false;}
bool SFE_UBLOX_GNSS::setNavigationFrequency(uint8_t navFreq, uint16_t maxWait){return false;}



SDClass SD{};
bool SDClass::exists(const char *fileName) {
    /*
       Returns true if the supplied file path exists.
    */
    return map.find(fileName) != map.end();
}
bool SDClass::begin(uint8_t csPin) {
    /*
      Performs the initialisation required by the sdfatlib library.
      Return true if initialization succeeds, false otherwise.
    */
   return true;
}
File SDClass::open(const char *file_path, uint8_t mode) {
    if (mode & O_CREAT) {
        if (map.find(file_path) != map.end()) {
            map.erase(file_path);
            map.emplace(file_path, FileStorage(mode));
            File temp = File(&map.find(file_path)->second);
            return temp;
        }
        else {
            map.emplace(file_path, FileStorage(mode));
            File temp = File(&map.find(file_path)->second);
            return temp;
        }
    }
    else {
        throw std::runtime_error("OTHER MODES NOT IMPLEMENTED");
    }
}

File::File() { storage_ = nullptr; }

File::File(FileStorage *storage) { storage_ = storage; }

void File::write(const uint8_t *data, size_t size) {
    const uint8_t *data_end = data + size;
    storage_->vect.insert(storage_->vect.end(), data, data_end);
}
void File::flush() {}
size_t File::println(const char *s) { 
    write((const uint8_t*)s, strlen(s));
    write((const uint8_t*)"\n", 1);
    return strlen(s);
}
