#ifndef BB8_CONFIG_H_
#define BB8_CONFIG_H_
/**
* Dome position and spin settings
*/
#define DOME_RIGHT_SERVO_PIN 5
#define DOME_LEFT_SERVO_PIN 4
#define DOME_SERVO_RAMP 3

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
#define S2S_LEAN_POT A0
#define S2S_LEAN_OFFSET -38 // Add to equal 512
#define S2S_ENABLE_DRV 33 // enable pin for S2S
#define S2S_LEFT_DRV 6 //FWD pin on motor controller
#define S2S_RIGHT_DRV 7 //REV pin on motor controller

/**
* Sound FX
**/
// sound board configuration
#define SFX_RST 37
#define SFX_SERIAL &Serial3
#define SFX_BAUD_RATE 9600
#define SFX_ACT_PIN 35
#define SFX_FADE_PIN A2

/**
* DEBUGGING FLAGS
* These exist in the headers of the project and are repeated here for usability.
**/
//#define DEBUG_CONTROL
#define DEBUG_DRIVE_MOVEMENT
//#define DEBUG_DOME_MOVEMENT
//#define DEBUG_SFX
//#define DEBUG_IMU

#endif //BB8_CONFIG_H_
