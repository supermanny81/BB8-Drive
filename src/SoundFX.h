#include "Adafruit_Soundboard.h"
#include "Arduino.h"
#include "Stream.h"

#ifndef SFX_H_
#define SFX_H_

// default settings, call setup to change these
#define _SFX_RST 37
#define _SFX_SERIAL &Serial3

class SoundFX {
  public:
    SoundFX() {;}
    ~SoundFX() {;}
    void setup(Stream* stream, int8_t sfx_reset);
    void playVoice();

  private:
    Adafruit_Soundboard sfx = Adafruit_Soundboard(_SFX_SERIAL, NULL, _SFX_RST);

};

#endif //SFX_H_
