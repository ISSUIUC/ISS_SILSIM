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
#include "ZOEM8Q0.hpp"

CpuStateContext context;
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

PWMServo::PWMServo() {}

LSM9DS1::LSM9DS1() {}

ZOEM8Q0::ZOEM8Q0() {}

MS5611::MS5611(uint8_t pin){}
void MS5611::init(){}
int MS5611::read(uint8_t bits){}
uint32_t MS5611::getPressure() const {}
int32_t MS5611::getTemperature() const {}

void SPIClass::begin() {}

