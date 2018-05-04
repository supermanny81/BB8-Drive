#include <Arduino.h>
#include <ArduinoLog.h>
#include <PID_v1.h>

#include "MovementUtils.h"

#ifndef Drive_H_
#define Drive_H_

#define DEBUG_DRIVE_MOVEMENT


// FWD/REV en 31
// FWD PIN  13
// REV PIN 12
class Drive {
  public:
    Drive() {}
    ~Drive() {;}

    void setup(uint8_t enablePin, uint8_t FWDPin, uint8_t REVPin) {
      this->enablePin = enablePin;
      this->FWDPin = FWDPin;
      this->REVPin = REVPin;
      // configure pins
      digitalWrite(enablePin, 0);
      pinMode(enablePin, OUTPUT);
      digitalWrite(FWDPin, 0);
      pinMode(FWDPin, OUTPUT);
      digitalWrite(REVPin, 0);
      pinMode(REVPin, OUTPUT);
    }

    void setSpeed(int16_t speed) {
      if (enabled) {
        targetSpeed = constrain(speed, -255, 255);
        if (!reversed)
          targetSpeed *= -1;
      } else {
        targetSpeed = 0;
      }
    }
    /**
    * Repeatably call this method to enable dome movement.
    */
    void task() {
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= 10) {
        move();
      }
      #ifdef DEBUG_DRIVE_MOVEMENT
        if (count == 500) {
          Log.notice(F("Drive:FWD/REV - En: %T, [T] %d, [A] %d\n"),
            this->enabled, this->targetSpeed, this->currentSpeed);
          count = 0;
        } else {
          count++;
        }
      #endif
    }

    bool isReversed() {
        return this->reversed;
    }

    void setReversed(bool reversed) {
      this->reversed = reversed;
    }

    bool isEnabled() {
      return this->enabled;
    }

    void setEnable(bool enabled) {
      this->enabled = enabled;
    }

  private:
    unsigned long previousMillis = 0; // used to determine if loop shoudl run
    uint8_t enablePin = 0, FWDPin = 0, REVPin = 0;
    boolean reversed = true, enabled = false;
    int16_t targetSpeed = 0, currentSpeed = 0;
    #ifdef DEBUG_DRIVE_MOVEMENT
      int count = 0;
    #endif
    #ifdef MAIN_DRIVE_RAMP
      const uint8_t DRIVE_RAMPING = MAIN_DRIVE_RAMP;
    #else
      const uint8_t DRIVE_RAMPING = 5;
    #endif

    void move() {
      currentSpeed = MovementUtils::ease(currentSpeed,
        targetSpeed, DRIVE_RAMPING);
      if (currentSpeed > 0){
        analogWrite(FWDPin, abs(currentSpeed));
        analogWrite(REVPin, 0);
      } else if (currentSpeed < 0) {
        analogWrite(FWDPin, 0);
        analogWrite(REVPin, abs(currentSpeed));
      } else {
        analogWrite(FWDPin, 0);
        analogWrite(REVPin, 0);
      }
      if (this->enabled)
        digitalWrite(enablePin, HIGH);
      else
        digitalWrite(enablePin, LOW);
    }

};
#endif //Drive_H_
