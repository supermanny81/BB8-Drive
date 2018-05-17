#ifndef Drive_H_
#define Drive_H_

#include <Arduino.h>
#include <ArduinoLog.h>
#include <PID_v1.h>

#include "MovementUtils.h"
#include "SmoothingUtils.h"

//#define DEBUG_DRIVE_MOVEMENT

#define _DRIVE_TASK_INTERVAL 10

class Drive {
  public:
    Drive() {}
    ~Drive() {;}

    void setup(uint8_t enablePin, uint8_t FWDPin, uint8_t REVPin,
        uint8_t tiltEnablePin, uint8_t LEFTPin, uint8_t RIGHTPin,
        uint8_t spinEnablePin, uint8_t spinLeftPin, uint8_t spinRightPin) {
      // drive
      this->enablePin = enablePin;
      this->FWDPin = FWDPin;
      this->REVPin = REVPin;
      // lean
      this->tiltEnablePin = tiltEnablePin;
      this->RIGHTPin = RIGHTPin;
      this->LEFTPin = LEFTPin;
      // spin
      this->spinEnablePin = spinEnablePin;
      this->spinLeftPin = spinLeftPin;
      this->spinRightPin = spinRightPin;
      // drive configure pins
      digitalWrite(this->enablePin, 0);
      pinMode(this->enablePin, OUTPUT);
      digitalWrite(this->FWDPin, 0);
      pinMode(this->FWDPin, OUTPUT);
      digitalWrite(this->REVPin, 0);
      pinMode(this->REVPin, OUTPUT);
      // s2s configure PINs
      digitalWrite(this->tiltEnablePin, 0);
      pinMode(this->tiltEnablePin, OUTPUT);
      digitalWrite(this->LEFTPin, 0);
      pinMode(this->LEFTPin, OUTPUT);
      digitalWrite(this->RIGHTPin, 0);
      pinMode(this->RIGHTPin, OUTPUT);
      // Flywheel PINs
      digitalWrite(this->spinEnablePin, 0);
      pinMode(this->spinEnablePin, OUTPUT);
      digitalWrite(this->spinLeftPin, 0);
      pinMode(this->spinLeftPin, OUTPUT);
      digitalWrite(this->spinRightPin, 0);
      pinMode(this->spinRightPin, OUTPUT);
      // S2S servo setup
      this->s2sServo.SetMode(AUTOMATIC);
      this->s2sServo.SetOutputLimits(-255, 255);
      this->s2sServo.SetSampleTime(20);
    }

    void setSpeed(int16_t speed) {
      if (enabled) {
        speed = constrain(speed, -255, 255);
        if (!reversed)
          speed *= -1;
        targetSpeed = SmoothingUtils::smooth(speed, .9, targetSpeed);
      } else {
        targetSpeed = 0;
      }
    }

    void setTilt(int16_t x) {
      if (enabled) {
        x = constrain(x, -90, 90);
        if (!reversed)
          x *= -1;
        setPoint_S2S = SmoothingUtils::smooth(x, .9, setPoint_S2S);
      } else {
        setPoint_S2S = 0;
      }
    }

    /**
    * Repeatably call this method to enabled drive movement.
    */
    void task() {
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= _DRIVE_TASK_INTERVAL) {
        move();
        spin();
        tilt();
      }
      #ifdef DEBUG_DRIVE_MOVEMENT
        if (count == 500) {
          Log.notice(F("Drive::task - FWD/REV (En: %T, T: %d, A: %d) "
              "S2S (P: %d, S: %D, I: %D, O: %D) "
              "SPIN (T: %d, A: %d)\n"),
            this->enabled, this->targetSpeed, this->currentSpeed, this->s2s_pot,
            this->setPoint_S2S, this->input_S2S, this->output_S2S,
            this->targetSpin, this->currentSpin);
          count = 0;
        } else {
          count++;
        }
      #endif
    }

    /**
    * Returns the drive's perspective on wether or not the BB8 front position
    * has been reveresd.
    */
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

    void setSpin(int16_t speed) {
      if (enabled) {
        speed = constrain(speed, -255, 255);
        targetSpin = SmoothingUtils::smooth(speed, .9, targetSpin);
        if (!reversed)
          targetSpin *= -1;
      } else {
        targetSpin = 0;
      }
    }

  private:
    unsigned long previousMillis = 0; // used to determine if loop shoudl run
    uint8_t enablePin = 0, FWDPin = 0, REVPin = 0, tiltEnablePin = 0,
      LEFTPin = 0, RIGHTPin = 0, spinEnablePin = 0, spinLeftPin = 0,
      spinRightPin =0;
    boolean reversed = true, enabled = false;
    int16_t targetSpeed = 0, currentSpeed = 0;
    int16_t targetSpin = 0, currentSpin = 0;
    int16_t s2s_pot = 0;
    //PID settings for the S2S tilt (S2S - Servo)
    double pk_S2S = 10.0, ik_S2S = 0.00, dk_S2S = 0.00;
    double setPoint_S2S = 0, input_S2S = 0, output_S2S = 0;
    PID s2sServo = PID(&input_S2S, &output_S2S, &setPoint_S2S,
      pk_S2S, ik_S2S , dk_S2S, DIRECT);

    #ifdef DEBUG_DRIVE_MOVEMENT
      int count = 0;
    #endif
    #ifdef MAIN_DRIVE_RAMP
      const uint8_t DRIVE_RAMPING = MAIN_DRIVE_RAMP;
    #else
      const uint8_t DRIVE_RAMPING = 5;
    #endif
    #ifdef S2S_LEAN_POT //override
      const uint8_t LEAN_POT = S2S_LEAN_POT;
    #else
      const uint8_t LEAN_POT = A0;
    #endif

    /**
    * Drives the BB unit FWD and Reverse
    */
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

    void spin() {
      currentSpin = MovementUtils::ease(currentSpin,
        targetSpin, DRIVE_RAMPING);
      if (currentSpin > 0){
        analogWrite(spinLeftPin, abs(currentSpin));
        analogWrite(spinRightPin, 0);
      } else if (currentSpin < 0) {
        analogWrite(spinLeftPin, 0);
        analogWrite(spinRightPin, abs(currentSpin));
      } else {
        analogWrite(spinLeftPin, 0);
        analogWrite(spinRightPin, 0);
      }
      if (this->enabled)
        digitalWrite(spinEnablePin, HIGH);
      else
        digitalWrite(spinEnablePin, LOW);
    }

    void tilt() {
      //  smooth our potentiometer readings
      s2s_pot = SmoothingUtils::smooth(analogRead(this->LEAN_POT), 0.45, s2s_pot);
      // TODO: Replace S2S_LEAN_MIN|MAX macros
      input_S2S = floor(SmoothingUtils::smooth(map(s2s_pot,
          S2S_LEAN_MIN, S2S_LEAN_MAX, -90, 90), 0.9, input_S2S));
      s2sServo.Compute();

      // send to motor controller
      if (output_S2S > 0) {
        analogWrite(LEFTPin, abs(output_S2S));
        analogWrite(RIGHTPin, 0);
      } else if (output_S2S < 0) {
        analogWrite(LEFTPin, 0);
        analogWrite(RIGHTPin, abs(output_S2S));
      } else {
        analogWrite(LEFTPin, 0);
        analogWrite(RIGHTPin, 0);
      }
      if (this->enabled)
        digitalWrite(this->tiltEnablePin, HIGH);
      else
        digitalWrite(this->tiltEnablePin, LOW);
    }
};
#endif //Drive_H_
