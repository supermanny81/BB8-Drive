#ifndef BB8_CONFIG_H_
#define BB8_CONFIG_H_

// get data types
#include <Arduino.h>

/**
* SoundFX configuration
*/
#define SFX_RST 37 // PIN
#define SFX_SERIAL &Serial3 // Serial Port

/**
* Dome position and spin settings
*/
#define DOME_RIGHT_SERVO_PIN 5
#define DOME_LEFT_SERVO_PIN 4
#define DOME_SERVO_RAMP 5
#define DOME_SPIN_PWM 3
#define DOME_SPIN_IN1 53
#define DOME_SPIN_IN2 49
#define DOME_SPIN_POT A4
#define DOME_SPIN_RAMP 15
#define DOME_SPIN_CENTER_POSITION 512

/**
* Main drive configuation.
*/
#define DRIVE_EN 29
#define MAIN_DRIVE_FWD 13
#define MAIN_DRIVE_REV 12


#endif //BB8_CONFIG_H_