#ifndef DomeMovement_H_
#define DomeMovement_H_

#include <Arduino.h>
#include <ArduinoLog.h>

#include "MovementUtils.h"
#include "VarSpeedServo.h"

//#define DEBUG_DOME_MOVEMENT

class DomeMovement {
  public:
    DomeMovement() {}
    ~DomeMovement() {;}

    /**
    * Setup the class.
    */
    void setup(uint8_t leftServoPin, uint8_t rightServoPin, uint8_t spinPWM,
      uint8_t spinIn1, uint8_t spinIn2, uint8_t domeSpinPot,
      uint16_t domeSpinCenterPos
      ) {
      this->spinPWM = spinPWM;
      this->spinIn1 = spinIn1;
      this->spinIn2 = spinIn2;
      this->domeSpinPot = domeSpinPot;
      this->domeSpinCenterPos = constrain(domeSpinCenterPos, 0, 1024);

      // X,Y movement for dome, leftServo is your left when you face
      // the drive
      leftServo.attach(leftServoPin);
      rightServo.attach(rightServoPin);
      // center dome (X,Y)
      leftServo.write(90, 30, false);
      rightServo.write(90, 30, false);
      leftServo.wait();
      rightServo.wait();

      // setup the pins for the L298N motor (dome spin)
      pinMode(this->spinPWM, OUTPUT);
      pinMode(this->spinIn1, OUTPUT);
      pinMode(this->spinIn2, OUTPUT);
    }

    /**
    * Repeatably call this method to enable dome movement.
    */
    void task() {
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= 10) {
        previousMillis = currentMillis;
        this->domeSpinPotPos = analogRead(this->domeSpinPot);
        this->centerDome();
        this->spin();
        this->move();
      }
      #ifdef DEBUG_DOME_MOVEMENT
        if (count == 500) {
          Log.notice(F("Dome::XY - Left: [T] %d [A] %d, Right: [T] %d [A] %d\n"),
            targetServo1, currentServo1, targetServo2, currentServo2);
          Log.notice(F("Dome::Spin - Pos: %F, Speed: %d, Center: %t\n"),
            this->getDomeSpinPosition(), this->targetRotationSpeed,
            this->center);
          count = 0;
        } else {
          count++;
        }
      #endif
    }

    void setDomeXY(int16_t x, int16_t y) {
      //x = x-3; //offset, the servo's spline teeth aren't very high res
      x = constrain(x, -50, 65); // can't hit the drive gear
      y = constrain(y, -70, 70); //

      if (reversed) {
        // invert the controls
        x = x * -1;
        y = y * -1;
      }
      // mix in X
      targetServo1 = constrain(map(y, -90, 90, 180, 0) + (x/2), 0, 180);
      targetServo2 = constrain(map(y, -90, 90, 0, 180) + (x/2), 0, 180);
    }

    float getDomeSpinPosition() {
      return this->domeSpinPotPos;
    }

    void setDomeSpin(int16_t speed) {
      this->targetRotationSpeed = constrain(speed, -255, 255);
    }

    void faceCenter() {
      this->center = true;
    }

    bool isReversed() {
        return this->reversed;
    }

    void setReversed(bool reversed) {
      this->reversed = reversed;
      if (domeSpinCenterPos >= 512) {
        domeSpinCenterPos -= 512;
      } else {
        domeSpinCenterPos += 512;
      }
      if (targetRotationSpeed == 0)
        faceCenter();
    }

  private:
    unsigned long previousMillis = 0; // used to determine if loop shoudl run
    VarSpeedServo leftServo;
    VarSpeedServo rightServo;
    int16_t targetServo1 = 90, targetServo2 = 90;
    int16_t currentServo1 = 90, currentServo2 = 90;

    // speed of motor rotation, -128..127, 0 is stop
    int16_t targetRotationSpeed = 0, currentRotationSpeed = 0;
    uint8_t spinPWM, spinIn1, spinIn2;
    uint8_t domeSpinPot;
    int16_t domeSpinPotPos;
    int16_t domeSpinCenterPos;
    bool center = false, reversed = true;
    #ifdef DOME_SPIN_RAMP
      const uint8_t SPIN_RAMPING = DOME_SPIN_RAMP;
    #else
      const uint8_t SPIN_RAMPING = 5;
    #endif
    #ifdef DEBUG_DOME_MOVEMENT
      int count = 0;
    #endif
    #ifdef DOME_SERVO_RAMP
      const uint8_t XY_RAMPING = DOME_SERVO_RAMP;
    #else
      const uint8_t XY_RAMPING = 5;
    #endif

    /**
    * Determines direction and speed (with ramping) of the dome
    */
    void spin() {
      currentRotationSpeed = MovementUtils::ease(currentRotationSpeed,
        targetRotationSpeed, SPIN_RAMPING);

      int16_t speed = currentRotationSpeed;
      if (this->reversed)
        speed *= -1;

      // set direction
      if (speed < 0) {
        digitalWrite(this->spinIn1, LOW);
        digitalWrite(this->spinIn2, HIGH);
      } else if (speed > 0) {
        digitalWrite(this->spinIn1, HIGH);
        digitalWrite(this->spinIn2, LOW);
      } else {
        digitalWrite(this->spinIn1, LOW);
        digitalWrite(this->spinIn2, LOW);
      }
      // remvoe the low range of the PWM pulses
      analogWrite(this->spinPWM,
        map(abs(speed), 0, 255, 75, 255));
    }

    void move() {
      // apply additional easing on servo travel
      currentServo1 = MovementUtils::ease(currentServo1, targetServo1, XY_RAMPING);
      currentServo2 = MovementUtils::ease(currentServo2, targetServo2, XY_RAMPING);
      leftServo.write(currentServo1, 80, false);
      rightServo.write(currentServo2, 80, false);
    }

    /**
    * Moves the dome back to center position
    */
    void centerDome() {
      if (this->center == true) {
        int16_t min_range = domeSpinCenterPos - 20;
        int16_t max_range = domeSpinCenterPos + 20;
        int16_t pot_pos = domeSpinPotPos;
        if (min_range < 0)
          min_range += 1024;
          max_range += 1024;
          pot_pos += 1024;
        if (!(pot_pos >= min_range
            && pot_pos <= max_range)) {
              // speed is partially determined by distance
              int16_t speed = domeSpinCenterPos - domeSpinPotPos;
          this->targetRotationSpeed = constrain(speed, -255, 255);
        } else {
          this->targetRotationSpeed = 0;
          this->center = false;
        }
      }
    }

};
#endif //DomeMovement_H_
