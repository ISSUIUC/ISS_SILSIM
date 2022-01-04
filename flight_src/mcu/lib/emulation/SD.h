//
// Created by 16182 on 10/3/2021.
//

#ifndef SILSIM_SD_H
#define SILSIM_SD_H

#include "Arduino.h"
#include<cstdint>

struct File {
    void write(const uint8_t * data, size_t size);
    void flush();
};

struct {
    static bool exists(const char * fileName){
        return false;
    }
} SD;

#endif  // SILSIM_SD_H
