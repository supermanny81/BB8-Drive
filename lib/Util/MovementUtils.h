#ifndef MovementUtils_H_
#define MovementUtils_H_

#include <Arduino.h>

class MovementUtils {

  public:
    /**
    * Performs easing, best if used over time
    */
    static int16_t ease(int16_t current, int16_t target, uint8_t ramping) {
      if (current < target) {
        if (target - current > (ramping + 1)) {
          current += ramping;
        } else {
          current = target;
        }
      } else if (current > target) {
        if (current - target > (ramping + 1) ) {
          current -= ramping;
        } else {
          current = target;
        }
      } else {
        current = target;
      }
      return current;
    }
};
#endif //MovementUtils
