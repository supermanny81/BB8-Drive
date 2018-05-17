#include <Arduino.h>
#include <ArduinoLog.h>
#include <SPI.h>
#include <XBOXRECV.h>

#include "BBConfig.h"
#include "Droid.h"
#include "Voltage.h"
#include "XBOX360.h"

//#define DEBUG_CONTROL

Droid droid = Droid();
Controller* controller = new XBOX360();
Voltage* voltage = Voltage::getInstance();

int count = 0;

/**
* Prepares the drive using settings from BBConfig.h and overrides from a custom
* MyConfig.h.
*/
void setup() {
  Serial.begin(115200);
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  Log.notice(F("BB8 Drive Control Started...\n"));

  droid.dome.setup(
    DOME_LEFT_SERVO_PIN,
    DOME_RIGHT_SERVO_PIN,
    DOME_SPIN_PWM,
    DOME_SPIN_IN1,
    DOME_SPIN_IN2,
    DOME_SPIN_POT,
    DOME_SPIN_CENTER_POSITION
  );

  droid.drive.setup(
    DRIVE_EN,
    MAIN_DRIVE_FWD,
    MAIN_DRIVE_REV,
    S2S_ENABLE_DRV,
    S2S_LEFT_DRV,
    S2S_RIGHT_DRV,
    FLYWHEEL_EN_DRV,
    FLYWHEEL_LEFT_DRV,
    FLYWHEEL_RIGHT_DRV
  );

  droid.sfx.setup(SFX_SERIAL, SFX_RST, SFX_BAUD_RATE);

  controller->setup(&droid);

  Log.notice(F("BB8 Drive Ready...\n"));
}

void loop() {
  voltage->sample();
  controller->task();
  droid.task();

  #ifdef DEBUG_CONTROL
  if (count == 500) {
    Log.notice(F("Control::IMU - Pitch: %F, Roll: %F\n"),
      droid.imu.data.pitch, droid.imu.data.roll);
    uint16_t contBatLvl = controller->getBatteryLevel();
    Log.notice(F("Control::Voltage - Drive: %F (%d%%), Xbox: %d (%d%%)\n"),
      voltage->getVCC(), voltage->getVCCPct(), contBatLvl,
      contBatLvl*100/4);
    count = 0;
  }
  count++;
  #endif
}
