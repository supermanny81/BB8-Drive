#ifndef Smoothing_H_H
#define Smoothing_H_H

#include <Arduino.h>

class SmoothingUtils {

  public:
    static float smooth(float data, float filterVal, float smoothedVal){
      if (filterVal > 1){      // check to make sure param's are within range
        filterVal = .99;
      }
      else if (filterVal <= 0){
        filterVal = 0;
      }
      smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);
      return (float)smoothedVal;
    }

};
#endif //SmoothingUtils
