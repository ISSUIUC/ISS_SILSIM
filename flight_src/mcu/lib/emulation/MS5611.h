//
// Created by 16182 on 1/2/2022.
//

#ifndef SILSIM_MS5611_H
#define SILSIM_MS5611_H

#include<cstdint>

struct MS5611 {
    MS5611(uint8_t pin);
    void init();
    int read(uint8_t bits = 8);
    uint32_t getPressure() const;
    int32_t getTemperature() const;

    double _temperature;
    double _pressure;
};

#endif  // SILSIM_MS5611_H
