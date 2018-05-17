#ifndef BB8_CONFIG_H_
#define BB8_CONFIG_H_
/**
*  DO NOT OVERRIDE VALUES IN THIS FILE!!!
*
*  This is a default reference for all of the possible configurations you can
*  change in your BB unit.  This file will change over time and it's defaults
*  will likely overwrite changes you have made.
*
*  To change any config items create a file called MyConfig.h in the same
*  folder as your .ino file.  Options copied there will override those defined
*  here and importantly will not overriden the next time you update the code.
*/


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
#define S2S_LEAN_MIN 360 // Add to equal Max POT reading leaning to the right
#define S2S_LEAN_MAX 580 // Max POT reading leaning to the right
#define S2S_ENABLE_DRV 33 // enable pin for S2S
#define S2S_LEFT_DRV 6 //FWD pin on motor controller
#define S2S_RIGHT_DRV 7 //REV pin on motor controller
#define FLYWHEEL_EN_DRV 29 // tied to main drive, but could easily be it's own
#define FLYWHEEL_LEFT_DRV 23
#define FLYWHEEL_RIGHT_DRV 25

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
//#define DEBUG_DRIVE_MOVEMENT
//#define DEBUG_DOME_MOVEMENT
//#define DEBUG_SFX
//#define DEBUG_IMU

#if defined(__has_include)
  #if __has_include("MyConfig.h")
    #include "MyConfig.h"
  #else
    #warning "       DO NOT OVERRIDE VALUES IN THIS FILE"
    #warning "--------------------------------------------------"
    #warning "A custom configuration header does not exist."
    #warning "Create a file called MyConfig.h and ovverride the "
    #warning "directives (config items) listed here."
  #endif
#endif

#endif //BB8_CONFIG_H_
