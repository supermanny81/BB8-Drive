#ifndef SFX_H_
#define SFX_H_

#include "Adafruit_Soundboard.h"
#include "Arduino.h"
#include "ArduinoLog.h"
#include "Stream.h"

#define DEBUG_SFX

// default settings, call setup to change these
#define _SFX_RST 37
#define _SFX_SERIAL &Serial3
#define _SFX_ACT_PIN 35
#define _SFX_FADE_PIN A2
#define _SFX_BAUD_RATE 9600

// internal settings
#define _SFX_TASK_INTERVAL 50

class SoundFX {
  public:
    SoundFX() {
    }
    ~SoundFX() {;}

    /**
    * Sets up the soundFX class.
    */
    void setup(HardwareSerial* serial = _SFX_SERIAL,
        int8_t sfx_reset = _SFX_RST,
        unsigned long baud_rate = _SFX_BAUD_RATE,
        uint8_t sfx_act_pin = _SFX_ACT_PIN,
        uint8_t sfx_fade_pin = _SFX_FADE_PIN
      ) {
      serial->begin(baud_rate);
      this->serial = serial;
      this->sfx_act_pin = sfx_act_pin;
      this->sfx_fade_pin = sfx_fade_pin;
      this->sfx_reset_pin = sfx_reset;

      pinMode(this->sfx_act_pin, INPUT);
      pinMode(this->sfx_fade_pin, INPUT);

      this->reset();
      Log.notice(F("SoundFX::setup - complete.\n"));
    }

    /**
    * Work.  Run this repeatidly in the main loop.
    */
    void task() {
      unsigned long currentMillis = millis();
      if (currentMillis - this->previousMillis >= _SFX_TASK_INTERVAL) {
        this->previousMillis = currentMillis;
        this->psiActivity();
      }
      #ifdef DEBUG_SFX
        if (count == 500) {
          count = 0;
          Log.notice(F("SoundFX::task - PSI EN %T, LVL %d\n"),
            this->psi_enabled, this->psi_level);
        } else {
          count++;
        }
      #endif
    }

    /**
    * Play a file by name and set the PSI to enabled|disabled
    */
    void playFile(char* filename, boolean psiEnabled = true) {
      Log.notice(F("SoundFX::playFile - %s\n"), filename);
      if (this->isSoundActive()) {
          this->serial->println('q');
          unsigned long current = millis();
          // much faster than using serial communication
          while(this->isSoundActive()) {
            if ((millis() - current) > 100) {
              Log.warning(F("SoundFX::playFile - Pin active too long.\n"));
              break;
            }
          }
      }
      this->serial->print('P');
      this->serial->println(filename);
      this->psi_enabled = psiEnabled;
      this->lastSound = millis();
    }

    /**
    * Always call by file name.
    */
    void playTrack(uint8_t bank, uint8_t track, boolean psiEnabled = true) {
      char _bank[2];
      _bank[0] = '0' + ((bank/10)%10);   // tens digit
      _bank[1] = '0' + (bank%10);        // ones digit
      String ftxt = "T";
      ftxt += _bank[0];
      ftxt += _bank[1];
      ftxt += F("NEXT");
      ftxt += track;
      ftxt += F("OGG"); // no dot in filename
      char* filename = const_cast<char*> (ftxt.c_str());
      this->playFile(filename, psiEnabled);
    }

    /**
    * Convience method to play a track in bank given a range.
    */
    void playTrack(uint8_t bank, uint8_t min, uint8_t max,
        boolean psiEnabled) {
      this->playTrack(bank, random(min, max), psiEnabled);
    }

    /**
    * Raise volume in increments of 10 (0...205 is the range)
    */
    void volUp() {
      for (uint8_t i = 0; i < 5; i++) {
        this->serial->println("+");
      }
      Log.notice(F("SoundFX::volUp() - volume up.\n"));
    }

    /**
    * Lower volume in increments of 10 (0...205 is the range)
    */
    void volDown() {
      for (uint8_t i = 0; i < 5; i++) {
        this->serial->println("-");
      }
      Log.notice(F("SoundFX::volDown - volume down.\n"));
    }

  private:
    unsigned long previousMillis = 0; // used to determine if loop shoudl run
    unsigned long lastSound = 0;
    HardwareSerial* serial;
    uint8_t sfx_act_pin, sfx_reset_pin, sfx_fade_pin;
    bool psi_enabled = false;
    uint8_t psi_level = 0;
    #ifdef DEBUG_SFX
      int count = 0;
    #endif

    /**
    * Checks the activity pin.
    */
    boolean isSoundActive() {
      if (digitalRead(this->sfx_act_pin) == LOW) {
        return true;
      }
      return false;
    }

    void psiActivity() {
      if ((millis() - this->lastSound) < 500 && this->psi_enabled) {
        readLevel();
      } else if (this->psi_enabled) {
        if (isSoundActive()) {
          readLevel();
        } else {
          this->psi_enabled = false;
          this->psi_level = 0;
        }
      }
    }

    /**
    * Read the level and apply and multiplier.  Sets the level between 0.255.
    */
    void readLevel() {
      this->psi_level = constrain(
        analogRead(this->sfx_fade_pin) * 10,
        0, 255);
    }

    void reset() {
      digitalWrite(this->sfx_reset_pin, LOW);
      pinMode(this->sfx_reset_pin, OUTPUT);
      delay(10);
      pinMode(this->sfx_reset_pin, INPUT);
      delay(1000); // give a bit of time to 'boot up'
    }
};

#endif //SFX_H_
