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
    if (value > 126) {
        value = 126;
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
    // a sine function
    float angle = asin(((length - (-0.0705732))/18.07384) - 0.958137) / 0.0160488;

    roundOffAngle(angle);

    // servo_cs rotates backwards
    servo_->write(50 + angle);

#ifdef SERVO_DEBUG
    Serial.print("\nclockwise: ");
    Serial.print(cw_angle);
    Serial.print(" counterclockwise: ");
    Serial.print(ccw_angle);
#endif
}

#endif