//#if defined(ARDUINO_ARCH_RP2040)

#include "Arduino.h"
#include "PDM.h"
#include "OpenPDMFilter.h"
#include "mbed_interface.h"

extern "C" {
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
}

#include "pdm.pio.h"

#define PIN_PDM_DIN 7
#define PIN_PDM_CLK 28
// Hardware peripherals used
uint dmaChannel = 3;
PIO pio = pio1;
uint sm = 0;

// PIO program offset
static uint offset;

// raw buffers contain PDM data
#define RAW_BUFFER_SIZE 512 // should be a multiple of (decimation / 8)
uint8_t rawBuffer0[RAW_BUFFER_SIZE];
uint8_t rawBuffer1[RAW_BUFFER_SIZE];
uint8_t* rawBuffer[2] = {rawBuffer0, rawBuffer1};
volatile int rawBufferIndex = 0; 
//uint8_t test[64][512] = 0;


#define HALF_BUFFER_SIZE 256 
uint8_t left_channel[HALF_BUFFER_SIZE];
int16_t left_tmp[32];
uint8_t right_channel[HALF_BUFFER_SIZE];
int16_t right_tmp[32];

//抽样因子，PDM频率 = 音频采样频率 * 抽取因子
//抽取因子通常处于48至128的范围之间，超频后会降到64
int decimation = 64;

// final buffer is the one to be filled with PCM data
int16_t* volatile finalBuffer;

// OpenPDM filter used to convert PDM into PCM
#define FILTER_GAIN     250
TPDMFilter_InitStruct filter;


// 声道判断
uint8_t channel_index;  //we use stereo as default.

extern "C" {
  __attribute__((__used__)) void dmaHandler(void)
  {
    PDM.IrqHandler(true);
  }
}


PDMClass::PDMClass(int dinPin, int clkPin, int pwrPin) :
  _dinPin(dinPin),
  _clkPin(clkPin),
  _pwrPin(pwrPin),
  _onReceive(NULL),
  _gain(-1),
  _channels(-1),
  _samplerate(-1),
  _init(-1)
{
}

PDMClass::~PDMClass()
{
}

int PDMClass::begin(Channel_Mode channels, int sampleRate)
{
  channel_index = channels; // only one channel available

  // clear the final buffers
  _doubleBuffer.reset();
  finalBuffer = (int16_t*)_doubleBuffer.data();
  int finalBufferLength = _doubleBuffer.availableForWrite() / sizeof(int16_t);
  _doubleBuffer.swap(0);

  // The mic accepts an input clock from 1.2 to 3.25 Mhz
  // Setup the decimation factor accordingly
  //if ((sampleRate * decimation * 2) > 3250000) {
    decimation = 64;
  //}

  // Sanity check, abort if still over 3.25Mhz
  //if ((sampleRate * decimation * 2) > 3250000) {
  //  mbed_error_printf("Sample rate too high, the mic would glitch\n");
  //  mbed_die();
  //}

  int rawBufferLength = RAW_BUFFER_SIZE / (decimation / 8);
  // Saturate number of samples. Remaining bytes are dropped.
  if (rawBufferLength > finalBufferLength) {
    rawBufferLength = finalBufferLength;
  }

  /* Initialize Open PDM library */
  filter.Fs = sampleRate;
  filter.MaxVolume = 1;
  if (channel_index == stereo) //mono
  {
    filter.nSamples = rawBufferLength / 2; // need to support 2 channel
  }
  else{
    filter.nSamples = rawBufferLength; // need to support 2 channel
  }
  filter.LP_HZ = sampleRate / 2;
  filter.HP_HZ = 10;
  filter.In_MicChannels = 1;
  filter.Out_MicChannels = 1;
  filter.Decimation = decimation;
  if(_gain == -1) {
    _gain = FILTER_GAIN;
  }
  filter.filterGain = _gain;
  Open_PDM_Filter_Init(&filter);

  // Configure PIO state machine
  float clkDiv = (float)clock_get_hz(clk_sys) / sampleRate / decimation / 2; //sys is 125MHz
  if (channel_index == left) //mono
  {
    if(pio_can_add_program(pio, &pdm_pio_left_program)) {
      offset = pio_add_program(pio, &pdm_pio_left_program);
      pdm_pio_left_program_init(pio, sm, offset, _clkPin, _dinPin, clkDiv);
    } 
    else {
      mbed_error_printf("Cannot load pio program\n");
      mbed_die();
    }    
  }
  else if (channel_index == right)  // mono
  {
    if(pio_can_add_program(pio, &pdm_pio_right_program)) {
      offset = pio_add_program(pio, &pdm_pio_right_program);
      pdm_pio_right_program_init(pio, sm, offset, _clkPin, _dinPin, clkDiv);
    } 
    else {
      mbed_error_printf("Cannot load pio program\n");
      mbed_die();
    }    
  }
  else  //stereo
  {
    if(pio_can_add_program(pio, &pdm_pio_program)) {
      offset = pio_add_program(pio, &pdm_pio_program);
      pdm_pio_program_init(pio, sm, offset, _clkPin, _dinPin, clkDiv);
    } 
    else {
      mbed_error_printf("Cannot load pio program\n");
      mbed_die();
    }
  }
  


  // Wait for microphone 
  delay(100);

  // Configure DMA for transferring PIO rx buffer to raw buffers
  dma_channel_config c = dma_channel_get_default_config(dmaChannel);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);

  // Clear DMA interrupts
  dma_hw->ints1 = 1u << dmaChannel; 
  // Enable DMA interrupts
  dma_channel_set_irq1_enabled(dmaChannel, true);
  irq_set_exclusive_handler(DMA_IRQ_1, dmaHandler);
  irq_set_enabled(DMA_IRQ_1, true);

  dma_channel_configure(dmaChannel, &c,
    rawBuffer[rawBufferIndex],        // Destinatinon pointer
    &pio->rxf[sm],      // Source pointer
    RAW_BUFFER_SIZE, // Number of transfers
    true                // Start immediately
  );

  _init = 1;

  return 1;
}

void PDMClass::end()
{
  if (channel_index == left) //mono
  {
      pio_remove_program(pio, &pdm_pio_left_program, offset);    
  }
  else if (channel_index == right) //mono
  {
      pio_remove_program(pio, &pdm_pio_right_program, offset);
  }
  else{
      pio_remove_program(pio, &pdm_pio_program, offset);
  }
  dma_channel_abort(dmaChannel);
  pinMode(_clkPin, INPUT);
  decimation = 128;
  rawBufferIndex = 0;
  offset = 0;
}

int PDMClass::available()
{
  NVIC_DisableIRQ(DMA_IRQ_1n);
  size_t avail = _doubleBuffer.available();
  NVIC_EnableIRQ(DMA_IRQ_1n);
  return avail;
}

int PDMClass::read(void* buffer, size_t size)
{
  NVIC_DisableIRQ(DMA_IRQ_1n);
  int read = _doubleBuffer.read(buffer, size);
  NVIC_EnableIRQ(DMA_IRQ_1n);
  return read;
}

void PDMClass::onReceive(void(*function)(void))
{
  _onReceive = function;
}

void PDMClass::setGain(int gain)
{
  _gain = gain;
  if(_init == 1) {
    filter.filterGain = _gain;
    Open_PDM_Filter_Init(&filter);
  }
}

void PDMClass::setBufferSize(int bufferSize)
{
  _doubleBuffer.setSize(bufferSize);
}

void channel_filter(uint8_t* buffer)
{
  size_t j = 0;
  for (size_t i = 0; i < 256; i++)
  {
    //left
    left_channel[i] = (buffer[j] & 0x80) | ((buffer[j] & 0x20) << 1) | ((buffer[j] & 0x08) << 2) | ((buffer[j] & 0x02) << 3) | 
                      ((buffer[j+1] & 0x80) >> 4) | ((buffer[j+1] & 0x20) >> 3) | ((buffer[j+1] & 0x08) >> 2) | ((buffer[j+1] & 0x02) >> 1);
    //right
    right_channel[i] = ((buffer[j] & 0x40) << 1) | ((buffer[j] & 0x10) << 2) | ((buffer[j] & 0x04) << 3) | ((buffer[j] & 0x01) << 4) | 
                      ((buffer[j+1] & 0x40) >> 3) | ((buffer[j+1] & 0x10) >> 2) | ((buffer[j+1] & 0x04) >> 1) | (buffer[j+1] & 0x01);    
    j += 2;
  }
 
}

static int16_t left_filter = 0;
static int16_t right_filter = 0;
static int8_t on_off = 0;
void PDMClass::IrqHandler(bool halftranfer)
{
  static int cutSamples = 100;
 
  // Clear the interrupt request.
  dma_hw->ints0 = 1u << dmaChannel; 
  // Restart dma pointing to the other buffer 
  int shadowIndex = rawBufferIndex ^ 1;
  dma_channel_set_write_addr(dmaChannel, rawBuffer[shadowIndex], true);

  if (_doubleBuffer.available()) {
    // buffer overflow, stop
    return end();
  }

  if (channel_index == stereo) //mono
  {
  //left and right filter
  channel_filter(rawBuffer[rawBufferIndex]);
  // fill final buffer with PCM samples
  if (filter.Decimation == 128) {
      //Open_PDM_Filter_128(rawBuffer[rawBufferIndex], finalBuffer, 1, &filter);
  } else {
    
      Open_PDM_Filter_64(left_channel, left_tmp, 1, &filter);
      Open_PDM_Filter_64(right_channel, right_tmp, 1, &filter);
  }
  if (on_off == 0)  // store
  {
    left_filter = left_tmp[31];
    right_filter = right_tmp[31]; 
    on_off = 1;
  }
  else  // curve is still not smooth, need linear interpolation
  {
    left_tmp[0] = left_filter + (left_tmp[3] - left_filter)/4;
    left_tmp[1] = left_tmp[0] + (left_tmp[3] - left_filter)/4;
    left_tmp[2] = left_tmp[1] + (left_tmp[3] - left_filter)/4;  
    right_tmp[0] = right_filter - (right_filter-right_tmp[3])/4;
    right_tmp[1] = right_tmp[0] - (right_filter-right_tmp[3])/4;
    right_tmp[2] = right_tmp[1] - (right_filter-right_tmp[3])/4;
    left_filter = left_tmp[31];
    right_filter = right_tmp[31];         
  }
  
    // combine left and right 
    for (int i = 0; i < 32; i++)
    {
      finalBuffer[i*2] = left_tmp[i];
      finalBuffer[i*2+1] = right_tmp[i];
    }
    if (cutSamples) {
      memset(finalBuffer, 0, cutSamples);
      cutSamples = 0;
    }
        // swap final buffer and raw buffers' indexes
    finalBuffer = (int16_t*)_doubleBuffer.data();
    _doubleBuffer.swap(filter.nSamples * 2 * sizeof(int16_t));
    rawBufferIndex = shadowIndex;
  }
  else{
     // fill final buffer with PCM samples
    if (filter.Decimation == 128) {
    //Open_PDM_Filter_128(rawBuffer[rawBufferIndex], finalBuffer, 1, &filter);
    } else {
      Open_PDM_Filter_64(rawBuffer[rawBufferIndex], finalBuffer, 1, &filter);
    }

    if (cutSamples) {
      memset(finalBuffer, 0, cutSamples);
      cutSamples = 0;
    } 
      // swap final buffer and raw buffers' indexes
      finalBuffer = (int16_t*)_doubleBuffer.data();
      _doubleBuffer.swap(filter.nSamples * sizeof(int16_t));
      rawBufferIndex = shadowIndex;
  }

    if (_onReceive) {
      _onReceive();
    }

  
}

PDMClass PDM(PIN_PDM_DIN, PIN_PDM_CLK, -1);

//#endif
