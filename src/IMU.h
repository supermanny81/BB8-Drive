#ifndef IMU_H_
#define IMU_H_

#include <Arduino.h>
#include <ArduinoLog.h>
#include "EasyTransfer.h"

// default settings, call setup to change these
#define _IMU_SERIAL &Serial2
#define _IMU_BAUD 115200

//#define DEBUG_IMU

class IMU {
  public:
    IMU() {
      setup(_IMU_SERIAL, _IMU_BAUD);
    }
    ~IMU() {;}

    struct RECEIVE_DATA_STRUCTURE_IMU{
      float IMUloop;
      float pitch;
      float roll;
    } data;

    byte status;

    /**
    * Setup the class.
    */
    void setup(HardwareSerial* serial, long baud_rate) {
      s = serial;
      s->begin(baud_rate);
      imuReciever.begin(details(data), s);
    }

    /**
    * Recieves the data from the IMU
    */
    void task(){
      if(millis() - lastReceivedMillis >= 10){
        if(s->available() > 0){
          imuReciever.receiveData();
          lastReceivedMillis = millis();
        }
        IMUtimeout();
      }
    }

  private:
    EasyTransfer imuReciever;
    unsigned long lastReceivedMillis;
    HardwareSerial* s;

    void IMUtimeout() {
      if(millis() - lastReceivedMillis >= 500){
        status = 1;
        Log.fatal(F("IMU::IMUTimeout - No data recieved in %s millis."),
          millis() - lastReceivedMillis);
      }else{
        if(status != 0){
          status = 0;
        }
      }
    }
};

#endif //IMU_H_
