#ifndef VOLTAGE_H_
#define VOLTAGE_H_

#include <ArduinoLog.h>

// defaults for a 10 cell NiMh battery on A3, if you are using something else,
// you need to call the setup function with the proper values
#define MIN_VOLTAGE 10.0
#define MAX_VOLTAGE 14.0
#define VOLTAGE_SENSOR_PIN A3

class Voltage {
  private:

    float _minVoltage = MIN_VOLTAGE;
    float _maxVoltage = MAX_VOLTAGE;
    int _pin = VOLTAGE_SENSOR_PIN;

    float volts = 0.00;
    byte percent = 0;
    Voltage() {};
    Voltage(Voltage const&); // copy disabled
    void operator=(Voltage const&); // assigment disabled

  public:
    static Voltage* getInstance() {
      static Voltage voltage;
      return &voltage;
    }

    void setup(int pin, float minVoltage, float maxVoltage) {
      _pin = pin;
      _minVoltage = minVoltage;
      _maxVoltage = maxVoltage;
    }

    void sample() {
      volts = SmoothingUtils::smooth((analogRead(_pin) / 1024.) * 25,
        .1, volts);
      percent = map(constrain(volts, _minVoltage, _maxVoltage),
                    _minVoltage, _maxVoltage, 0, 100);
    }

    float getVCC() {
      return volts;
    }

    byte getVCCPct() {
      return percent;
    }
};
#endif
