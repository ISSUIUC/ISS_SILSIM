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
void KX134::update_data() {}
int16_t KX134::binary_to_decimal(int16_t) { return 0; }
void KX134::init() {}
int16_t KX134::get_x_accel_raw() { return 0; }
int16_t KX134::get_y_accel_raw() { return 0; }
int16_t KX134::get_z_accel_raw() { return 0; }
float KX134::get_x_gforce() { return 0; }
float KX134::get_y_gforce() { return 0; }
float KX134::get_z_gforce() { return 0; }
float KX134::get_x_accel() { return 0; }
float KX134::get_y_accel() { return 0; }
float KX134::get_z_accel() { return 0; }

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

}
void LSM9DS1::readAccel() {}
float LSM9DS1::calcGyro(int16_t gyro) { return 0; }
float LSM9DS1::calcAccel(int16_t accel) { return 0; }
void LSM9DS1::setAccelScale(uint8_t aScl) {}
void LSM9DS1::readMag() {}
float LSM9DS1::calcMag(int16_t mag) { return 0; }

MS5611::MS5611(uint8_t pin){}
void MS5611::init(){}
int MS5611::read(uint8_t bits) {
    //_temperature = context.barometer_pointer ...
    _pressure = context.barometer_pointer->get_data();
    }
    uint32_t MS5611::getPressure() const { return _pressure * 100; } //millibar to pascal conversion
int32_t MS5611::getTemperature() const { return 0; }

SPIClass SPI{};
void SPIClass::begin() {}

bool SFE_UBLOX_GNSS::getPVT(uint16_t maxWait){return false;}
int32_t SFE_UBLOX_GNSS::getLatitude(uint16_t maxWait){return 0;}
int32_t SFE_UBLOX_GNSS::getLongitude(uint16_t maxWait){return 0;}
int32_t SFE_UBLOX_GNSS::getAltitude(uint16_t maxWait){return 0;}
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
