//
// Created by 16182 on 10/3/2021.
//

#ifndef SILSIM_ARDIUINO_H
#define SILSIM_ARDIUINO_H

#include<cstdint>
#define HIGH 1
#define LOW 0
#define OUTPUT 1

#define LED_BUILTIN 13

struct SerialClass{
    void println(const char *);
    void begin(uint32_t frequency);
    void write(void * data, uint32_t size);
    bool operator!();
    operator bool();
};

void digitalWrite(uint8_t pin, uint8_t val);
void pinMode(uint8_t pin, uint8_t mode);
void delay(uint32_t ms);

#endif  // SILSIM_ARDIUINO_H
