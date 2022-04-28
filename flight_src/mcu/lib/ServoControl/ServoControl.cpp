#ifndef SERVO_CPP
#define SERVO_CPP

#include "ServoControl.h"

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
    // These are correcting factors for finding the angle. We still need to
    // calculate what these values are. These are placeholders for now. m and
    // offset should include the radian to degree conversion.
    float m = (1.0 / radius) * (180.0 / (3.1415));  // degrees / meter
    float offset = 0;

    // This is the actual conversion from the inputted length to the angles
    // desired for the servos.
    float angle = (length * m + offset);

    roundOffAngle(angle);

    // servo_cs rotates backwards
    servo_->write(126 - angle);

#ifdef SERVO_DEBUG
    Serial.print("\nclockwise: ");
    Serial.print(cw_angle);
    Serial.print(" counterclockwise: ");
    Serial.print(ccw_angle);
#endif
}

#endif