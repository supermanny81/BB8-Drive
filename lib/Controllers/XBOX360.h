#ifndef XBOX360_H_
#define XBOX360_H_
#include <ArduinoLog.h>
#include <SPI.h>
#include <USB.h>
#include <XBOXRECV.h>

#include "Controller.h"
#include "Droid.h"

class XBOX360: public Controller {
  public:
    void setup(Droid* droid) override {
      this->droid = droid;
      if (usb.Init() == -1) {
        Log.fatal(F("ERROR:\n"));
        Log.fatal(F("================================\n"));
        Log.fatal(F("Failed to initialize the USB board.\n"
                    "Check all connections and run the \n"
                    "'board_qc' example sketch included \n"
                    "in the USB library to verify the HW.\n\n"
                  ));
        while (1); //halt
      }
    }

    void task() override {
      usb.Task();
      this->processInput();
    }

    uint8_t getBatteryLevel() override {
      return xbox.getBatteryLevel(0);
    }

  private:
    USB usb;
    XBOXRECV xbox = XBOXRECV(&usb);
    Droid* droid;
    bool firstConnect = true;

    void processInput() {
      if (xbox.XboxReceiverConnected) {
          int i = 0; // controller number
          if (xbox.Xbox360Connected[i]) {
            if (firstConnect) {
              this->isEnabled(false);
              this->firstConnect = false;
            }
            // DOME|BALL SPIN
            if (xbox.getButtonPress(L2, i) || xbox.getButtonPress(R2, i)) {
                int16_t l2 = xbox.getButtonPress(L2, i);
                int16_t r2 = xbox.getButtonPress(R2, i);
                if (xbox.getButtonPress(L1) || xbox.getButtonPress(R1)) {
                  if (l2 > r2) {
                    droid->drive.setSpin(l2*-1);
                  } else {
                    droid->drive.setSpin(r2);
                  }
                  droid->dome.setDomeSpin(0);
                } else {
                  if (l2 > r2) {
                    droid->dome.setDomeSpin(l2*-1);
                  } else {
                    droid->dome.setDomeSpin(r2);
                  }
                  droid->drive.setSpin(0);
                }
            } else {
              droid->dome.setDomeSpin(0);
              droid->drive.setSpin(0);
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
                // Tilt L/R
                rx = xbox.getAnalogHat(RightHatX, i);
                if (rx < -7500) {
                  rx = map(rx, -32768, -7500, -90, 0);
                } else if (rx >= 7500) {
                  rx = map(rx, 7500, 32767, 0, 90);
                } else {
                  rx = 0;
                }
              }
              if (xbox.getAnalogHat(RightHatY, i) > 7500 || xbox.getAnalogHat(RightHatY, i) < -7500) {
                // Drive FWD/REV
                ry = xbox.getAnalogHat(RightHatY, i);
                // map Y account for the deadzone
                if (ry < -7500) {
                  ry = map(ry, -32768, -7500, -255, 0);
                } else if (ry >= 7500) {
                  ry = map(ry, 7500, 32767, 0, 255);
                } else {
                  ry = 0;
                }
              }
              droid->dome.setDomeXY(lx, ly);
              droid->drive.setSpeed(ry);
              droid->drive.setTilt(rx);
            } else {
              droid->dome.setDomeXY(0, 0);
              droid->drive.setSpeed(0);
              droid->drive.setTilt(0);
            }
            // Volume UP|DOWN
            if (xbox.getButtonClick(UP, i)) {
              droid->sfx.volUp();
            }
            if (xbox.getButtonClick(DOWN, i)) {
              droid->sfx.volDown();
            }
            // enable/disable the drive
            if (xbox.getButtonClick(START, i)) {
              this->isEnabled(!droid->drive.isEnabled());
            }
            if (xbox.getButtonClick(BACK, i)) {
              droid->dome.setReversed(!droid->dome.isReversed());
              droid->drive.setReversed(!droid->drive.isReversed());
            }
            if (xbox.getButtonClick(L3, i)) {
              // move dome to face forward
              droid->dome.faceCenter();
            }
            if (xbox.getButtonClick(SYNC, i)) {
              // turn off controller
              xbox.disconnect(i);
            }
            // XYAB - SOUNDS
            if (xbox.getButtonClick(X)) {
              if (xbox.getButtonPress(R1)) {
                  droid->sfx.playTrack(4, 0, 9, true);
              } else if (xbox.getButtonPress(L1)) {
                droid->sfx.playTrack(0, 0, 9, true);
              } else {
                droid->sfx.playTrack(6, 0, true);
              }
            }
            if (xbox.getButtonClick(Y)) {
              if (xbox.getButtonPress(R1)) {
                droid->sfx.playTrack(5, 1, false);
              } else {
                droid->sfx.playTrack(1, 0, 9, true);
              }
            }
            if (xbox.getButtonClick(A)) {
              if (xbox.getButtonPress(R1)) {
                droid->sfx.playTrack(5, 4, false);
              } else {
                droid->sfx.playTrack(2, 0, 9, true);
              }
            }
            if (xbox.getButtonClick(B)) {
              if (xbox.getButtonPress(R1)) {
                droid->sfx.playTrack(5, 0, false);
              } else {
                droid->sfx.playTrack(3, 0, 9, true);
              }
            }

          } else {
              this->isEnabled(false);
              this->firstConnect = true;
          }
      } else {
        this->isEnabled(false);
        this->firstConnect = true;
      }
    }

    /**
    * Enable or disable the drive
    */
    void isEnabled(bool enable) {
      droid->drive.setEnable(enable);
      if (!droid->drive.isEnabled()) {
        xbox.setLedMode(ROTATING, 0);
      } else {
        xbox.setLedOn(LED1, 0);
      }
    }

};
#endif // XBOX360_H_
