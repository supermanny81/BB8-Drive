#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "Droid.h"
#include "ArduinoLog.h"

/**
* An abstract class meant to be extended with a complete implementation for the
* controller of your choice.
**/
class Controller {

public:
    virtual void setup(Droid* droid) {
      Log.fatal(F("ERROR:\n"));
      Log.fatal(F("================================\n"));
      Log.fatal(F("You are in the base class. The \n"
                  "controller hash not been initialized.\n\n"
                ));
      while (1); //halt
    }

    virtual void task() {;};
    virtual uint8_t getBatteryLevel() {return 0;};
};
#endif // CONTROLLER_H_
