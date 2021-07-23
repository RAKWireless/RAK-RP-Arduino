/**
 * @file Soil_Conductivity_Monitoring.ino
 * @author rakwireless.com
 * @brief This sketch demonstrate reading soil conductivity values
 *    and send the data to lora gateway.
 * @version 0.1
 * @date 2020-07-28
 * @copyright Copyright (c) 2020
 */
#include <Arduino.h>
#include "LoRaWan-Arduino.h" //http://librarymanager/All#SX126x
#include <SPI.h>

#include <stdio.h>

#include "mbed.h"
#include "rtos.h"
#include <ModbusRTU.h>  //http://librarymanager/All#modbus-esp8266

ModbusRTU mb;
uint16_t coils[20];

bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void* data) 
{
  Serial.print("Request result: 0x");
  Serial.print(event, HEX);  
  return true;
}

using namespace std::chrono_literals;
using namespace std::chrono;

bool doOTAA = true;   // OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60                      /**< Maximum number of events in the scheduler queue. */

#define LORAWAN_DATERATE DR_0
#define LORAWAN_TX_POWER TX_POWER_0
#define JOINREQ_NBTRIALS 3 /**< Number of trials for the join request. */
DeviceClass_t g_CurrentClass = CLASS_A;
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_EU868;    /* Region:EU868*/
lmh_confirm g_CurrentConfirm = LMH_UNCONFIRMED_MSG;
uint8_t g_AppPort = LORAWAN_APP_PORT;

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
 */
static lmh_param_t g_lora_param_init = {
  LORAWAN_ADR_ON,
  LORAWAN_DATERATE,
  LORAWAN_PUBLIC_NETWORK,
  JOINREQ_NBTRIALS,
  LORAWAN_TX_POWER,
  LORAWAN_DUTYCYCLE_OFF
};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t g_lora_callbacks = {
  BoardGetBatteryLevel,
  BoardGetUniqueId,
  BoardGetRandomSeed,
  lorawan_rx_handler,
  lorawan_has_joined_handler,
  lorawan_confirm_class_handler,
  lorawan_join_failed_handler
};

//OTAA keys !!!! KEYS ARE MSB !!!!
uint8_t nodeDeviceEUI[8] = {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x33, 0x33};
uint8_t nodeAppEUI[8] = {0xB8, 0x27, 0xEB, 0xFF, 0xFE, 0x39, 0x00, 0x00};
uint8_t nodeAppKey[16] = {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88};

// ABP keys
uint32_t nodeDevAddr = 0x260116F8;
uint8_t nodeNwsKey[16] = {0x7E, 0xAC, 0xE2, 0x55, 0xB8, 0xA5, 0xE2, 0x69, 0x91, 0x51, 0x96, 0x06, 0x47, 0x56, 0x9D, 0x23};
uint8_t nodeAppsKey[16] = {0xFB, 0xAC, 0xB6, 0x47, 0xF3, 0x58, 0x45, 0xC7, 0x50, 0x7D, 0xBF, 0x16, 0x8B, 0xA8, 0xC1, 0x7C};

// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 64                     /**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 20000                        /**< Defines for user timer, the application data transmission interval. 20s, value in [ms]. */
static uint8_t g_m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];        //< Lora user application data buffer.
static lmh_app_data_t g_m_lora_app_data = {g_m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.

TimerEvent_t g_appTimer;
static uint32_t timers_init(void);

static uint32_t g_count = 0;
static uint32_t g_count_fail = 0;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial)
  {
    if ((millis() - timeout) < 5000)
    {
      delay(100);
    }
    else
    {
      break;
    }
  }
  // Initialize LoRa chip.
  lora_rak11300_init();

  Serial.println("=====================================");
  Serial.println("Welcome to RAK11300 LoRaWan!!!");
  if (doOTAA)
  {
    Serial.println("Type: OTAA");
  }
  else
  {
    Serial.println("Type: ABP");
  }

  switch (g_CurrentRegion)
  {
    case LORAMAC_REGION_AS923:
      Serial.println("Region: AS923");
      break;
    case LORAMAC_REGION_AU915:
      Serial.println("Region: AU915");
      break;
    case LORAMAC_REGION_CN470:
      Serial.println("Region: CN470");
    break;
  case LORAMAC_REGION_CN779:
    Serial.println("Region: CN779");
    break;
  case LORAMAC_REGION_EU433:
    Serial.println("Region: EU433");
    break;
  case LORAMAC_REGION_IN865:
    Serial.println("Region: IN865");
    break;
  case LORAMAC_REGION_EU868:
    Serial.println("Region: EU868");
    break;
  case LORAMAC_REGION_KR920:
    Serial.println("Region: KR920");
    break;
  case LORAMAC_REGION_US915:
    Serial.println("Region: US915");
    break;
  case LORAMAC_REGION_RU864:
    Serial.println("Region: RU864");
    break;
  case LORAMAC_REGION_AS923_2:
    Serial.println("Region: AS923-2");
    break;
  case LORAMAC_REGION_AS923_3:
    Serial.println("Region: AS923-3");
    break;
  case LORAMAC_REGION_AS923_4:
    Serial.println("Region: AS923-4");
    break;
  }
  Serial.println("=====================================");

  //  Init Modbus
  Serial1.begin(9600, SERIAL_8N1);
  mb.begin(&Serial1);
  mb.setBaudrate(9600);
  mb.master();
 
  //creat a user timer to send data to server period
  uint32_t err_code;

  err_code = timers_init();
  if (err_code != 0)
  {
    Serial.printf("timers_init failed - %d\n", err_code);
    return;
  }
  
  // Setup the EUIs and Keys
  if (doOTAA)
  {
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);
  }
  else
  {
    lmh_setNwkSKey(nodeNwsKey);
    lmh_setAppSKey(nodeAppsKey);
    lmh_setDevAddr(nodeDevAddr);
  }

  // Initialize LoRaWan
  err_code = lmh_init(&g_lora_callbacks, g_lora_param_init, doOTAA, g_CurrentClass, g_CurrentRegion);
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }

  // Start Join procedure
  lmh_join();
}

void loop2()
{
  uint32_t i = 0;
  short raw_conductivity;
  short raw_temperature;
  short raw_humidity;

  /* RS485 Power On */
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);
  delay(100);
  /* RS485 Power On */

  raw_conductivity = get_soil_conductivity();
  raw_temperature = get_soil_temperature();
  raw_humidity = get_soil_humidity();

  /* RS485 Power Off */
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, LOW);
  delay(100);
  /* RS485 Power Off */

  Serial.printf("-----raw_conductivity = %d-------\n", raw_conductivity);
  Serial.printf("-----raw_temperature = %d-------\n", raw_temperature);
  Serial.printf("-----raw_humidity = %d-------\n", raw_humidity);

  g_m_lora_app_data.port = g_AppPort;
  g_m_lora_app_data.buffer[i++] = 0x03;
  g_m_lora_app_data.buffer[i++] = (raw_conductivity >> 8) & 0xFF;
  g_m_lora_app_data.buffer[i++] = raw_conductivity & 0x00FF;
  g_m_lora_app_data.buffer[i++] = (raw_temperature >> 8) & 0xFF;
  g_m_lora_app_data.buffer[i++] = raw_temperature & 0x00FF;
  g_m_lora_app_data.buffer[i++] = (raw_humidity >> 8) & 0xFF;
  g_m_lora_app_data.buffer[i++] = raw_humidity & 0x00FF;

  g_m_lora_app_data.buffsize = i;

  delay(10000);
}

void loop()
{
  // Put your application tasks here, like reading of sensors,
  // Controlling actuators and/or other functions. 
  loop2();
}

/**@brief LoRa function for handling HasJoined event.
 */
void lorawan_has_joined_handler(void)
{
  if(doOTAA == true)
  {
    Serial.println("OTAA Mode, Network Joined!");
  }
  else
  {
    Serial.println("ABP Mode");
  }
  lmh_error_status ret = lmh_class_request(g_CurrentClass);
  if (ret == LMH_SUCCESS)
  {
    delay(1000);
    TimerSetValue(&g_appTimer, LORAWAN_APP_INTERVAL);
    TimerStart(&g_appTimer);
  }
}
/**@brief LoRa function for handling OTAA join failed
*/
static void lorawan_join_failed_handler(void)
{
  Serial.println("OTAA join failed!");
  Serial.println("Check your EUI's and Keys's!");
  Serial.println("Check if a Gateway is in range!");
}
/**@brief Function for handling LoRaWan received data from Gateway
 *
 * @param[in] app_data  Pointer to rx data
 */
void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
          app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
  // Informs the server that switch has occurred ASAP
  g_m_lora_app_data.buffsize = 0;
  g_m_lora_app_data.port = g_AppPort;
  lmh_send(&g_m_lora_app_data, g_CurrentConfirm);
}

static short read_reg(int reg_address)
{
  short reg_value;

 if (!mb.slave()) 
 {
  // If the request succeeds, the sensor sends the readings, that are
    // stored in the holding registers. The read() method can be used to
    // get the raw humidity temperature values.
   reg_value = mb.readHreg(1, reg_address, coils, 1, cbWrite);
   Serial.printf("ReadSensor from modbus is:%d\r\n",reg_value);
 }
 else
 {
    Serial.printf("modbus is error%d\r\n");
 }
  return reg_value;
}

static short get_soil_conductivity(void)
{
  return read_reg(0x0015);
}

static short get_soil_temperature(void)
{
  return read_reg(0x0003);
}

static short get_soil_humidity(void)
{
  return read_reg(0x0002);
}

void send_lora_frame(void)
{
  if (lmh_join_status_get() != LMH_SET)
  {
    //Not joined, try again later
    return;
  }

  lmh_error_status error = lmh_send(&g_m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS)
  {
    g_count++;
    Serial.printf("lmh_send ok count %d\n", g_count);
  }
  else
  {
    g_count_fail++;
    Serial.printf("lmh_send fail count %d\n", g_count_fail);
  }
}

/**@brief Function for handling user timerout event.
 */
void tx_lora_periodic_handler(void)
{
  TimerSetValue(&g_appTimer, LORAWAN_APP_INTERVAL);
  TimerStart(&g_appTimer);
  Serial.println("Sending frame now...");
  send_lora_frame();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
uint32_t timers_init(void)
{
  TimerInit(&g_appTimer, tx_lora_periodic_handler);
  return 0;
}
