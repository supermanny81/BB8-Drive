#include <Arduino.h>
#include <ArduinoLog.h>
#include <EasyTransfer.h>
#include <EEPROMex.h>
#include <PID_v1.h>
#include <SPI.h>
#include <XBOXRECV.h>

#include "BB8Config.h"
#include "DomeMovement.h"
#include "Drive.h"
#include "IMU.h"
#include "SoundFX.h"
#include "Voltage.h"

DomeMovement dome = DomeMovement();
Drive drive = Drive();
IMU imu = IMU();
SoundFX sfx = SoundFX();
USB usb;
XBOXRECV xbox(&usb);
Voltage* voltage = Voltage::getInstance();

int count = 0;

void setup() {
  Serial.begin(115200);
  dome.setup(
    DOME_LEFT_SERVO_PIN,
    DOME_RIGHT_SERVO_PIN,
    DOME_SPIN_PWM,
    DOME_SPIN_IN1,
    DOME_SPIN_IN2,
    DOME_SPIN_POT,
    DOME_SPIN_CENTER_POSITION
  );
  drive.setup(
    DRIVE_EN,
    MAIN_DRIVE_FWD,
    MAIN_DRIVE_REV
  );

  if (usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }

  // setup sound
  Serial3.begin(9600);
  sfx.setup(SFX_SERIAL, SFX_RST);

  // Initialize with log level and log output.
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  Log.notice(F("BB8 Drive Control Started...\n"));

}

void loop() {
  usb.Task();
  voltage->sample();
  imu.task();
  dome.task();
  drive.task();

  if (xbox.XboxReceiverConnected) {
    for (uint8_t i = 0; i < 4; i++) {
      if (xbox.Xbox360Connected[i]) {
        // DOME SPIN
        if (xbox.getButtonPress(L2, i) || xbox.getButtonPress(R2, i)) {
            int16_t l2 = xbox.getButtonPress(L2, i);
            int16_t r2 = xbox.getButtonPress(R2, i);
            if (l2 > r2) {
              dome.setDomeSpin(l2*-1);
            } else {
              dome.setDomeSpin(r2);
            }
        } else {
          dome.setDomeSpin(0);
        }
        // DOME XY and DRIVE
        if (xbox.getAnalogHat(LeftHatX, i) > 7500 ||
            xbox.getAnalogHat(LeftHatX, i) < -7500 ||
            xbox.getAnalogHat(LeftHatY, i) > 7500 ||
            xbox.getAnalogHat(LeftHatY, i) < -7500 ||
            xbox.getAnalogHat(RightHatX, i) > 7500 ||
            xbox.getAnalogHat(RightHatX, i) < -7500 ||
            xbox.getAnalogHat(RightHatY, i) > 7500 ||
            xbox.getAnalogHat(RightHatY, i) < -7500) {
          int16_t lx = 0, ly =0, rx = 0, ry = 0;
          if (xbox.getAnalogHat(LeftHatX, i) > 7500 || xbox.getAnalogHat(LeftHatX, i) < -7500) {
            lx = xbox.getAnalogHat(LeftHatX, i);
            // get X, remove the deadband and map it to the correcvt range
            if (lx < -7500) {
              lx = map(lx, -32768, -7500, -90, 0);
            } else if (lx >= 7500) {
              lx = map(lx, 7500, 32767, 0, 90);
            } else {
              lx = 0;
            }
          }
          if (xbox.getAnalogHat(LeftHatY, i) > 7500 || xbox.getAnalogHat(LeftHatY, i) < -7500) {
            // get Y, remove the deadband and map it the correct range
            ly = xbox.getAnalogHat(LeftHatY, i);
            if (ly < -7500) {
              ly = map(ly, -32768, -7500, -90, 0);
            } else if (ly >= 7500) {
              ly = map(ly, 7500, 32767, 0, 90);
            } else {
              ly = 0;
            }
          }
          if (xbox.getAnalogHat(RightHatX, i) > 7500 || xbox.getAnalogHat(RightHatX, i) < -7500) {
            rx = xbox.getAnalogHat(RightHatX, i);
            Serial.print(rx);
          }
          if (xbox.getAnalogHat(RightHatY, i) > 7500 || xbox.getAnalogHat(RightHatY, i) < -7500) {
            ry = xbox.getAnalogHat(RightHatY, i);
            // map Y account for the deadzone
            if (ry < -7500) {
              ry=map(ry, -32768, -7500, -255, 0);
            } else if (ry >= 7500) {
              ry=map(ry, 7500, 32767, 0, 255);
            } else {
              ry = 0;
            }
          }
          dome.setDomeXY(lx, ly);
          drive.setSpeed(ry);
        } else {
          dome.setDomeXY(0, 0);
          drive.setSpeed(0);
        }

        if (xbox.getButtonClick(UP, i)) {
          xbox.setLedOn(LED1, i);
          Serial.println(F("Up"));
        }
        if (xbox.getButtonClick(DOWN, i)) {
          xbox.setLedOn(LED4, i);
          Serial.println(F("Down"));
        }
        if (xbox.getButtonClick(LEFT, i)) {
          xbox.setLedOn(LED3, i);
          Serial.println(F("Left"));
        }
        if (xbox.getButtonClick(RIGHT, i)) {
          xbox.setLedOn(LED2, i);
          Serial.println(F("Right"));
        }
        // enable/disable the drive
        if (xbox.getButtonClick(START, i)) {
          drive.setEnable(!drive.isEnabled());
          if (!drive.isEnabled()) {
            xbox.setLedMode(ROTATING, 0);
          } else {
            xbox.setLedOn(LED1, 0);
          }
        }
        if (xbox.getButtonClick(BACK, i)) {
          dome.setReversed(!dome.isReversed());
        }
        if (xbox.getButtonClick(L3, i)) {
          // move dome to face forward
          dome.faceCenter();
        }
        if (xbox.getButtonClick(R3, i)) {
          Serial.println(F("R3"));
        }
        if (xbox.getButtonClick(SYNC, i)) {
          // turn off controller
          xbox.disconnect(i);
        }
      }
    }
  }

  if (count == 500) {
    Log.notice(F("Control::IMU - Pitch: %F, Roll: %F\n"),
      imu.data.pitch, imu.data.roll);
    uint16_t xboxBattery = xbox.getBatteryLevel(0);
    Log.notice(F("Control::Voltage - Drive: %F (%d%%), Xbox: %d (%d%%)\n"),
      voltage->getVCC(), voltage->getVCCPct(), xboxBattery,
      xboxBattery*100/4);
    count = 0;
  }
  count++;
}
