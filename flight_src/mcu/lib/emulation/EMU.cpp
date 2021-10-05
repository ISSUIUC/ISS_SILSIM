//
// Created by 16182 on 10/5/2021.
//
#include "Arduino.h"
#include "ChRt.h"
#include "KX134-1211.h"
#include "PWMServo.h"
#include "SparkFunLSM9DS1.h"
#include "ZOEM8Q0.hpp"

SerialClass Serial{};

void SerialClass::println(const char * str){
    puts(str);
}

void digitalWrite(int pin, int level){}

KX134::KX134() {}

PWMServo::PWMServo() {}

LSM9DS1::LSM9DS1() {}

ZOEM8Q0::ZOEM8Q0() {}
