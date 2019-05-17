#ifndef MY_CONFIG_H_
#define MY_CONFIG_H_

/**
* Find the mid point of your pot, then make sure the min and max are equal
* distance from center
*/
// ABSOLUTE MAX 590 MIN 390
#undef S2S_LEAN_MIN
#define S2S_LEAN_MIN 385 // Add to equal Max POT reading leaning to the right
#undef S2S_LEAN_MAX
#define S2S_LEAN_MAX 575 // Max POT reading leaning to the right

/**
* DEBUGGING FLAGS
* These exist in the headers of the project and are repeated here for usability.
**/
//#define DEBUG_CONTROL
#define DEBUG_DRIVE_MOVEMENT
//#define DEBUG_DOME_MOVEMENT
//#define DEBUG_SFX
//#define DEBUG_IMU

#endif //MY_CONFIG_H_
