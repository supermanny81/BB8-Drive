#ifndef Drive_H_
#define Drive_H_

#include <Arduino.h>
#include <ArduinoLog.h>
#include <PID_v1.h>

#include "IMU.h"
#include "MovementUtils.h"
#include "SmoothingUtils.h"

//#define DEBUG_DRIVE_MOVEMENT

#define _DRIVE_TASK_INTERVAL 15

class Drive {
  public:
    Drive() {}
    ~Drive() {;}

    void setup(IMU* imu, uint8_t enablePin, uint8_t FWDPin, uint8_t REVPin,
        uint8_t tiltEnablePin, uint8_t LEFTPin, uint8_t RIGHTPin,
        uint8_t spinEnablePin, uint8_t spinLeftPin, uint8_t spinRightPin) {
      this->imu = imu;
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
      this->s2sServo.SetSampleTime(15);
      // S2S stability setup
      this->s2sStability.SetMode(AUTOMATIC);
      this->s2sStability.SetOutputLimits(-90, 90);
      this->s2sStability.SetSampleTime(15);
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

    void setTilt(int16_t x) {
      if (enabled) {
        sp_stability = constrain(x, -90, 90);
        if (!reversed)
          sp_stability *= -1;
      } else {
        sp_stability = 0;
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
        if (count == 100) {
          Log.notice(F("Drive::task - FWD/REV (e: %T, t: %d, a: %d) "
              "IMU (p: %D, r: %D) "
              "SBL (s: %D, i: %D, o: %D) "
              "S2S (p: %d, s: %D, i: %D, o: %D) "
              "SPIN (t: %d, a: %d)\n"),
            this->enabled, this->targetSpeed, this->currentSpeed,
            imu->data.pitch, imu->data.roll,
            this->sp_stability, this->input_stability, this->output_stability,
            this->s2s_pot, this->setPoint_S2S, this->input_S2S, this->output_S2S,
            targetSpin, currentSpin);
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
        targetSpin = constrain(speed, -255, 255);
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
    int16_t s2s_speed = 0;
    IMU* imu;

    //PID settings for the S2S tilt (S2S - Servo)
    // 13, 0, .3 are Joe's values, after some tuning, I ended up here too, then
    // changed it again and agian
    // or maybe 14, 0, 0.0 ???
    //double pk_S2S = 9, ik_S2S = 0.03, dk_S2S = 0.05;
    double pk_S2S = 10, ik_S2S = 0.035, dk_S2S = 0.05;
    double setPoint_S2S = 0, input_S2S = 0, output_S2S = 0;
    PID s2sServo = PID(&input_S2S, &output_S2S, &setPoint_S2S,
      pk_S2S, ik_S2S , dk_S2S, DIRECT);

    //PID setttings for S2S stability
    double pK_stability = .95, iK_stability = .05,  dK_stability = 0.4;
    //double pK_stability = .6, iK_stability = 0, dK_stability = .35;
    double sp_stability = 0, input_stability = 0, output_stability = 0;
    PID s2sStability = PID(&input_stability, &output_stability, &sp_stability,
      pK_stability, iK_stability, dK_stability, DIRECT);

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
    #ifdef S2S_LEAN_MIN
      uint16_t S2S_POT_MIN = S2S_LEAN_MIN;
    #else
      uint16_t S2S_POT_MIN = 500;
    #endif
    #ifdef S2S_LEAN_MAX
      uint16_t S2S_POT_MAX = S2S_LEAN_MAX;
    #else
      uint16_t S2S_POT_MAX = 500;
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
      // stability
      input_stability = map(imu->data.roll, -25, 25, -90, 90);
      //input_stability = SmoothingUtils::smooth(input_stability,
      //  .05, input_stability);
      s2sStability.Compute();
      setPoint_S2S = constrain(output_stability, -90, 90);

      //  smooth our potentiometer readings
      s2s_pot = SmoothingUtils::smooth(analogRead(this->LEAN_POT), .05, s2s_pot);
      //s2s_pot = analogRead(this->LEAN_POT);
      // TODO: Replace S2S_LEAN_MIN|MAX macros
      input_S2S = floor(map(s2s_pot,
          S2S_POT_MIN, S2S_POT_MAX, -90, 90));
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
