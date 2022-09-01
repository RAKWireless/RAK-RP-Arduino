/*
   I2S stereo microphone (input) example
   Run using the Arduino Serial Plotter to see waveform.
   Released to the Public Domain by Earle F. Philhower, III

   For the Google AIY Voice Hat Microphone daughterboard, part
   of the Raspberry Pi AIY cardboard box, the I2S stereo pinout
   looking at the board top with the RPI logo on the left hand
   side:
            +--   ------------------------------------  --+
   left RPI | (1) GND (2) DIN (3) BCLK (4) LRCLK (5) 3.3V | AIY right
       logo +---------------------------------------------+ logo
*/

#include <I2S.h>

#define 

I2S i2s(INPUT);

// GPIO pin numbers
#define pBCLK 8
#define pWS (pBCLK+1)
#define pDIN 20

void setup() {
  Serial.begin(115200);

  i2s.setDATA(pDIN);
  i2s.setBCLK(pBCLK); // LRCLK = pBCLK+1
  i2s.setBitsPerSample(16);
  i2s.setFrequency(22050);
  i2s.begin();

  while (1) {
    int16_t l, r;
    i2s.read16(&l, &r);
    Serial.printf("%d %d\n", l, r);
  }
}

void loop() {
  /* Nothing here */
}
