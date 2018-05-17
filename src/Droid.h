#ifndef DROID_H_
#define DROID_H_

#include "DomeMovement.h"
#include "Drive.h"
#include "SoundFX.h"
#include "IMU.h"

/**
*  This class serves as a container for the various subsystems in a BB unit.
*/
class Droid {
  public:
    DomeMovement dome;
    Drive drive;
    IMU imu;
    SoundFX sfx;

    void task() {
      imu.task();
      dome.task();
      drive.task();
      sfx.task();
    }
};
#endif //DROID_H_
