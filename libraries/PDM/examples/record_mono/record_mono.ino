/*
  This example reads audio data from the on-board PDM microphones, and prints
  out the samples to the Serial console. The Serial Plotter built into the
  Arduino IDE can be used to plot the audio data (Tools -> Serial Plotter)

  Circuit:
  - Arduino Nano 33 BLE board, or
  - Arduino Nano RP2040 Connect, or
  - Arduino Portenta H7 board plus Portenta Vision Shield

  This example code is in the public domain.
*/

#include <PDM.h>
#include "SPI.h"
#include "SD.h"//http://librarymanager/All#SD

File myFile;
#define SAMPLE_SIZE   64
#define SAMPLE_DATA_LENGTH 512
#define PCM_HEAD_SIZE 44
#define BIT_PER_SAMPLE 16
#define RECORD_TIME    4
// default number of output channels
static const char channels = 1;

// default PCM output frequency
static const int frequency = 16000;

// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[SAMPLE_SIZE];
short tmp[SAMPLE_SIZE];
short totalBuffer[SAMPLE_DATA_LENGTH][SAMPLE_SIZE];

// Number of audio samples read
int samplesRead = 0;
int first_store = 0;
String pcm_format = "RIFF";
uint32_t data_size;


void add_head()
{
  uint8_t tmp = 0;
  //RIFF CHUNK
  uint32_t total_size = SAMPLE_DATA_LENGTH*SAMPLE_SIZE*2+PCM_HEAD_SIZE-8; 
  tmp = total_size & 0x000000FF;
  pcm_format += char(tmp);
  tmp = (total_size & 0x0000FF00)>>8;  
  pcm_format += char(tmp);
  tmp = (total_size & 0x00FF0000)>>16;  
  pcm_format += char(tmp);     
  tmp = (total_size & 0xFF000000)>>24;  
  pcm_format += char(tmp);

  pcm_format+="WAVE";      

  //Format CHUNK
  pcm_format+="fmt ";
  tmp = 16;
  pcm_format+=char(tmp);
  tmp = 0;
  pcm_format+=char(tmp);
  pcm_format+=char(tmp);
  pcm_format+=char(tmp);
  tmp = 1;
  pcm_format+=char(tmp);
  tmp = 0;
  pcm_format+=char(tmp);
  tmp = 1; //channel
  pcm_format+=char(tmp);
  tmp = 0;
  pcm_format+=char(tmp);
  
  uint32_t fre = frequency;  //little,
  tmp = fre & 0x000000FF;
  pcm_format += char(tmp);
  tmp = (fre & 0x0000FF00)>>8;  
  pcm_format += char(tmp);
  tmp = (fre & 0x00FF0000)>>16;  
  pcm_format += char(tmp);     
  tmp = (fre & 0xFF000000)>>24;  
  pcm_format += char(tmp);
  uint32_t ByteRate = frequency*channels*BIT_PER_SAMPLE/8;  //little, ByteRate is 32000, 播放速度换算成字节单位就是32000(字节/秒)。
  tmp = ByteRate & 0x000000FF;
  pcm_format += char(tmp);
  tmp = (ByteRate & 0x0000FF00)>>8;  
  pcm_format += char(tmp);
  tmp = (ByteRate & 0x00FF0000)>>16;  
  pcm_format += char(tmp);    
  tmp = (ByteRate & 0xFF000000)>>24;  
  pcm_format += char(tmp);  
  uint16_t BlockAlign = channels*BIT_PER_SAMPLE/8;      //little, BlockAlign is 4, here should be 0x0400
  tmp = BlockAlign & 0x000000FF;
  pcm_format += char(tmp);
  tmp = (BlockAlign & 0x0000FF00)>>8;  
  pcm_format += char(tmp);
  
  tmp = 16;
  pcm_format+=char(tmp);
  tmp = 0;
  pcm_format+=char(tmp);  


  //Data CHUNK
  pcm_format += "data";      
  uint32_t a_size = SAMPLE_DATA_LENGTH*SAMPLE_SIZE*2; //little
  tmp = a_size & 0x000000FF;
  pcm_format += char(tmp);
  tmp = (a_size & 0x0000FF00)>>8;  
  pcm_format += char(tmp);
  tmp = (a_size & 0x00FF0000)>>16;  
  pcm_format += char(tmp);   
  tmp = (a_size & 0xFF000000)>>24;  
  pcm_format += char(tmp);   
  myFile.print(pcm_format);
  //myFile.close(); 
  Serial.println("head finish!"); 
}

//little store
void store_data()
{
  uint8_t tmp = 0;
  for(int i=0;i<SAMPLE_DATA_LENGTH;i++)
  {
    pcm_format = "";
    for(int j=0;j<SAMPLE_SIZE;j++)
    {
      tmp = totalBuffer[i][j] & 0x00FF;
      pcm_format += char(tmp);   
      tmp = (totalBuffer[i][j] & 0xFF00) >> 8;
      pcm_format += char(tmp);   
    }
    myFile.print(pcm_format);
  }
  myFile.close();
}

void setup() {
  pinMode(WB_IO2,OUTPUT);
  digitalWrite(WB_IO2, HIGH);
  pinMode(PIN_LED1,OUTPUT);
  Serial.begin(115200);
  while (!Serial);

  if (!SD.begin()) 
  {    
    Serial.println("Card Mount Failed! Please make sure the card is inserted!");
    return;
  }

  // Check to see if the file exists:
  if (SD.exists("record.wav")) {
    Serial.println("Removing record.wav...");
    SD.remove("record.wav");    
  }
  delay(1000);
  // open a new file and immediately close it:
  Serial.println("Creating record.wav...");
  myFile = SD.open("record.wav", FILE_WRITE);
  add_head();
  
  // Configure the data receive callback
  PDM.onReceive(onPDMdata);
  Serial.println("Ready to say something! 3!");
  delay(1000);
  Serial.println("Ready to say something! 2!");  
  delay(1000);
  Serial.println("Ready to say something! 1!"); 
  delay(1000);     
  // Optionally set the gain
  // Defaults to 20 on the BLE Sense and 24 on the Portenta Vision Shield
  // PDM.setGain(30);

  // Initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate for the Arduino Nano 33 BLE Sense
  // - a 32 kHz or 64 kHz sample rate for the Arduino Portenta Vision Shield
  if (!PDM.begin(right, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

}



void loop() {

  if(samplesRead == SAMPLE_DATA_LENGTH && first_store == 0)
  {
    Serial.println("Record finish!");
    digitalWrite(PIN_LED1, HIGH);
    samplesRead = 0;
    first_store = 1;
    Serial.println("Store begin!");
    store_data();
    Serial.println("Store finish!");
  }
  digitalWrite(PIN_LED1, LOW);
}

/**
 * Callback function to process the data from the PDM microphone.
 * NOTE: This callback is executed as part of an ISR.
 * Therefore using `Serial` to print messages inside this function isn't supported.
 * */
void onPDMdata() {
  // Query the number of available bytes
  //int bytesAvailable = PDM.available();
  //Serial.println("onPDMdata");
  // Read into the sample buffer

  PDM.read(sampleBuffer, sizeof(sampleBuffer));
  if(samplesRead <= SAMPLE_DATA_LENGTH && first_store == 0)
  {
    memcpy(totalBuffer[samplesRead],sampleBuffer,sizeof(sampleBuffer));
    samplesRead++;
  }  
}
