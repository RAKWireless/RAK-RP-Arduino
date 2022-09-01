/**
   @file RAK11300_play_16s44.ino
   @author rakwireless.com
   @brief play samplerate 44100Hz 16 bits wav file audio datas
   @version 0.1
   @date 2022-05-20
   @copyright Copyright (c) 2022
**/
#include <I2S.h>
#include "Arduino.h"
#include "sound.h"

// Create the I2S port using a PIO state machine
I2S i2s(OUTPUT);
// GPIO pin numbers
#define pBCLK 8
#define pWS (pBCLK+1)
#define pDOUT 20

const int sampleRate = 44100; // sample rate in Hz

int count = 0;
int16_t i2s_data = 0;
int16_t *p_blox = &i2s_data;
int i=0;

uint8_t tx_flag = 0;
uint8_t rx_flag = 0;

void i2s_tx_irq(void);
void i2s_rx_irq(void);

void setup()
{   
  pinMode(WB_IO2,OUTPUT);
  digitalWrite(WB_IO2,HIGH); 
  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial)
  {
    if ((millis() - timeout) < 3000)
    {
      delay(100);
    }
    else
    {
      break;
    }
  }   
  Serial.println("=====================================");
  Serial.println("RAK11300 audio Test.");
  Serial.println("=====================================");  

  i2s.setBCLK(pBCLK);
  i2s.setDATA(pDOUT);
  i2s.setBitsPerSample(16); 
 // start I2S at the sample rate with 16-bits per sample
  if (!i2s.begin(sampleRate)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }
  count = sizeof(sample_buff)>>1;
  Serial.printf("sum count: %d \r\n",count);
}
void loop()
{  
  for(i=22;i<count;i++)
  {
    ((uint8_t *)p_blox)[0] = sample_buff[i*2];
    ((uint8_t *)p_blox)[1] = sample_buff[i*2+1];
      i2s.write(i2s_data); 
  } 
//  i2s.end();
  delay(5000);
}
