#ifndef SERVO_CPP
#define SERVO_CPP

#include "ServoControl.h"
#include <cmath>

/**
 * @brief A function to keep the value sent to the servo between 0 and 180
 * degrees.
 *
 * @param value The value determined by the control algorithm.
 */
ServoControl::ServoControl(PWMServo* servo) {
    servo_ = servo;
}
// TODO check values for max
void ServoControl::roundOffAngle(float& value) {
    if (value > 189) {
        value = 189;
    }
    if (value < 0) {
        value = 0;
    }
}

/**
 * @brief Takes the length of the flap extension and converts to angles for the
 * servo.
 *
 * @param length_one The length of the flap extension for the counterclockwise
 * flaps.
 * @param length_two The length of the flap extension for the clockwise flaps.
 *
 */
void ServoControl::servoActuation(float length) {
    // The angle is found through utilizing a fft and mapping extension/angle values to 
    // a sine function. len (mm), pass in ang (rad)
    
    float angle = 136.050812741891 - 62.3098522547825*asin(0.0553285866373617*(length * 1000) + 0.00390471397714149);

    roundOffAngle(angle);

    // servo_cs rotates backwards
    servo_->write(angle);

#ifdef SERVO_DEBUG
    Serial.print("\nclockwise: ");
    Serial.print(cw_angle);
    Serial.print(" counterclockwise: ");
    Serial.print(ccw_angle);
#endif
}

#endif