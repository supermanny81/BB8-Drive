#include "Arduino.h"
#include "Adafruit_Soundboard.h"
#include "SoundFX.h"

void SoundFX::setup(Stream* serial, int8_t sfx_reset) {
  sfx = Adafruit_Soundboard(serial, NULL, sfx_reset);
}

void SoundFX::playVoice() {

}
