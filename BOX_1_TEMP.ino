/**
 * @file LoRaWAN_OTAA_ABP.ino
 * @author rakwireless.com
 * @brief LoRaWan node example with OTAA/ABP registration
 * @version 0.1
 * @date 2020-08-21
 * 
 * @copyright Copyright (c) 2020
 * 
 * @note RAK4631 GPIO mapping to nRF52840 GPIO ports
   RAK4631    <->  nRF52840
   WB_IO1     <->  P0.17 (GPIO 17)
   WB_IO2     <->  P1.02 (GPIO 34)
   WB_IO3     <->  P0.21 (GPIO 21)
   WB_IO4     <->  P0.04 (GPIO 4)
   WB_IO5     <->  P0.09 (GPIO 9)
   WB_IO6     <->  P0.10 (GPIO 10)
   WB_SW1     <->  P0.01 (GPIO 1)
   WB_A0      <->  P0.04/AIN2 (AnalogIn A2)
   WB_A1      <->  P0.31/AIN7 (AnalogIn A7)
 */
#include <Arduino.h>
#include <LoRaWan-RAK4630.h> //http://librarymanager/All#SX126x
#include <SPI.h>
#include <Wire.h>
#include "SparkFun_SHTC3.h" //Click here to get the library: http://librarymanager/All#SparkFun_SHTC3
#include "Melopero_RV3028.h" //Click here to get the library: http://librarymanager/All#Melopero_RV3028
#include <rg15arduino.h>

Melopero_RV3028 rtc;            // Declare an instance of the RTC class
SHTC3 mySHTC3;                  // Declare an instance of the SHTC3 class
RG15Arduino rg15;               // Declare an instance of the RG15 class

// RAK4630 supply two LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

/* battery stuff */
#define PIN_VBAT WB_A0

uint32_t vbat_pin = PIN_VBAT;

#define VBAT_MV_PER_LSB (0.73242188F) // 3.0V ADC range and 12 - bit ADC resolution = 3000mV / 4096
#define VBAT_DIVIDER_COMP (1.73)      // Compensation factor for the VBAT divider, depend on the board

#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)


/* LoRa stuff */
bool doOTAA = true;   // OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60										  /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_0									  /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5							/*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 3										  /**< Number of trials for the join request. */
DeviceClass_t g_CurrentClass = CLASS_A;					/* class definition*/
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_EU868;    /* Region:EU868*/
lmh_confirm g_CurrentConfirm = LMH_CONFIRMED_MSG;				  /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;							        /* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t g_lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);

static float readVBAT(void);
static uint8_t mvToPercent(float mvolts);
static uint8_t mvToLoRaWanBattVal(float mvolts);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t g_lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                        lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler, lorawan_join_failed_handler
                                       };
//OTAA keys !!!! KEYS ARE MSB !!!!
uint8_t nodeDeviceEUI[8] = {0x60, 0xC5, 0xA8, 0xFF, 0xFE, 0x79, 0x96, 0xDB};
uint8_t nodeAppEUI[8] = {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09};
uint8_t nodeAppKey[16] = {0x64, 0x7D, 0x56, 0xE3, 0x5E, 0xBC, 0x36, 0xB8, 0xC4, 0x05, 0x2F, 0x3F, 0xC5, 0x13, 0xA8, 0x72};

// ABP keys
uint32_t nodeDevAddr = 0x260116F8;
uint8_t nodeNwsKey[16] = {0x7E, 0xAC, 0xE2, 0x55, 0xB8, 0xA5, 0xE2, 0x69, 0x91, 0x51, 0x96, 0x06, 0x47, 0x56, 0x9D, 0x23};
uint8_t nodeAppsKey[16] = {0xFB, 0xAC, 0xB6, 0x47, 0xF3, 0x58, 0x45, 0xC7, 0x50, 0x7D, 0xBF, 0x16, 0x8B, 0xA8, 0xC1, 0x7C};

// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 64                    /**< buffer size of the data to be transmitted. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];            //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.
static uint32_t count = 0;
static uint32_t count_fail = 0;

/* timer stuff */
#define FAIRUSE_TTN_INTERVAL      1000*1      /* with TTN dont go below 35s send interval (for payed services it will be ok to send faster) */
#define LORAWAN_APP_INTERVAL_A    1000*60*13  /* A timer: 1000ms * x (Temperatur & Humidity) Every 13min */
#define LORAWAN_APP_INTERVAL_B    1000*60*14  /* B timer: 1000ms * x (Rainsensor)            Every 14min */
#define LORAWAN_APP_INTERVAL_C    1000*60*15  /* C timer: 1000ms * x (Batterie)              Every 15min */
#define LOOP_INTERVAL             1000*10
#define DEVICE_NUMBER             0x01

TimerEvent_t appTimer_A;
TimerEvent_t appTimer_B;
TimerEvent_t appTimer_C;

static uint32_t timers_init(void);

/* semaphore for deep sleep in mainloop */
SemaphoreHandle_t loopEnable;
SemaphoreHandle_t loraSend;

/* = BATTERY ================================================================================================================================= */

/**
 * @brief Get RAW Battery Voltage
 */
float readVBAT(void)
{
    float raw;

    // Get the raw 12-bit, 0..3000mV ADC value
    raw = analogRead(vbat_pin);

    return raw * REAL_VBAT_MV_PER_LSB;
}

/**
 * @brief Convert from raw mv to percentage
 * @param mvolts
 *    RAW Battery Voltage
 */
uint8_t mvToPercent(float mvolts)
{
    if (mvolts < 3300)
        return 0;

    if (mvolts < 3600)
    {
        mvolts -= 3300;
        return mvolts / 30;
    }

    mvolts -= 3600;
    return 10 + (mvolts * 0.15F); // thats mvolts /6.66666666
}

/**
 * @brief get LoRaWan Battery value
 * @param mvolts
 *    Raw Battery Voltage
 */
uint8_t mvToLoRaWanBattVal(float mvolts)
{
    if (mvolts < 3300)
        return 0;

    if (mvolts < 3600)
    {
        mvolts -= 3300;
        return mvolts / 30 * 2.55;
    }

    mvolts -= 3600;
    return (10 + (mvolts * 0.15F)) * 2.55;
}

/* = SETUP ================================================================================================================================= */
/* entry point after booting */

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(LED_BUILTIN2, OUTPUT);
  digitalWrite(LED_BUILTIN2, LOW);
  

  // Initialize LoRa chip.
  lora_rak4630_init();

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
  Serial.println("=====================================");
  Serial.println("Welcome to RAK4630 LoRaWan!!!");
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
  }
  Serial.println("=====================================");

  Wire.begin();
  /* shtc3 init */
  Serial.println("shtc3 init");
  Serial.print("Beginning sensor."); // Most SHTC3 functions return a variable of the type "SHTC3_Status_TypeDef" to indicate the status of their execution
  mySHTC3.begin();               // To start the sensor you must call "begin()", the default settings use Wire (default Arduino I2C port)
  Wire.setClock(400000);             // The sensor is listed to work up to 1 MHz I2C speed, but the I2C clock speed is global for all sensors on that bus so using 400kHz or 100kHz is recommended
  Serial.println();

  if (mySHTC3.passIDcrc) // Whenever data is received the associated checksum is calculated and verified so you can be sure the data is true
  {            // The checksum pass indicators are: passIDcrc, passRHcrc, and passTcrc for the ID, RH, and T readings respectively
    Serial.print("ID Passed Checksum. ");
    Serial.print("Device ID: 0b");
    Serial.println(mySHTC3.ID, BIN); // The 16-bit device ID can be accessed as a member variable of the object
  }
  else
  {
    Serial.println("ID Checksum Failed. ");
  }
  
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
  
  // Initialize RTC Module
  rtc.initI2C(); // First initialize and create the rtc device

  rtc.writeToRegister(0x35,0x00);  
  rtc.writeToRegister(0x37,0xB4); //Direct Switching Mode (DSM): when VDD < VBACKUP, switchover occurs from VDD to VBACKUP
  
  rtc.set24HourMode();  // Set the device to use the 24hour format (default) instead of the 12 hour format

  // Set the date and time
  // year, month, weekday, date, hour, minute, second
  // Note: time is always set in 24h format
  // Note: month value ranges from 1 (Jan) to 12 (Dec)
  // Note: date value ranges from 1 to 31
  rtc.setTime(2022, 5, 3, 18, 16, 14, 0);

  // Start Join procedure
  lmh_join();

  Serial.println("RAK4631 Battery checker init");
  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);
  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14
  // Let the ADC settle
  delay(1);
  // Get a single ADC sample and throw it away
  readVBAT();

  // Initialize Serial for debug output
  timeout = millis();
  Serial1.begin(9600);
  
  while (!Serial1)
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
  rg15.setStream(&Serial1);
  Serial.println("=====================================");
  Serial.println("Serial 1 ist bereit");

 
  

  rg15.restartDevice();
  delay(10000);
  
  rg15.setContinuous();
  Serial.println("Set RG-15 in Continuous Mode");
  
  /* polls and updates values and if successful, returns true.
  if(rg15.poll()) 
  { 
    Serial.print(rg15.acc); 
  }*/
  
  
  /* create a semaphore */
  loopEnable = xSemaphoreCreateBinary();
  xSemaphoreGive(loopEnable);
  xSemaphoreTake(loopEnable, (TickType_t)10);

  loraSend = xSemaphoreCreateBinary();
  xSemaphoreGive(loraSend);
  
}

void loop()
{
  // Put your application tasks here, like reading of sensors,
  // Controlling actuators and/or other functions. 

    /* update RG15 values  
    if (rg15.poll())
    {
      acc = rg15.acc;
      eventAcc = rg15.eventAcc;
      totalAcc = rg15.totalAcc;
    }
    delay(1000);
    */
    
  //Serial.println("RAK4631:- I am going for a Nap (waiting for Semaphore)");

  // Ausgabe des Datums im RTC Modul
  Serial.printf("%d:%d:%d %d/%d/%d \n",rtc.getHour(),rtc.getMinute(),rtc.getSecond(),rtc.getYear(),rtc.getMonth(),rtc.getDate());
  delay(1000);

  //digitalWrite(LED_BUILTIN2, LOW);  // turn the LED on (HIGH is the voltage level)
  /* try to get the Semaphore */
  xSemaphoreTake(loopEnable, (TickType_t)LOOP_INTERVAL);
  delay(500);

  // Reset the Accumulation
  if(rtc.getHour() == 00 && rtc.getMinute() == 00)
    {
    rg15.resetAccumulation();
    Serial.println("Accumulation resetet");
    }

    
  /* woke up - show LED */
  //digitalWrite(LED_BUILTIN2, HIGH); // turn the LED off by making the voltage LOW
                         // wait for a second

  //Serial.println("RAK4631:- Hey I just Woke up (got Semaphore)");
      
}

/* ============================================================================================================================================== */

/**@brief LoRa function for handling HasJoined event.
 */
void lorawan_has_joined_handler(void)
{
  Serial.println("OTAA Mode, Network Joined!");

  lmh_error_status ret = lmh_class_request(g_CurrentClass);
  if (ret == LMH_SUCCESS)
  {
    delay(1000);
    TimerSetValue(&appTimer_A, LORAWAN_APP_INTERVAL_A);
    TimerSetValue(&appTimer_B, LORAWAN_APP_INTERVAL_B);
    TimerSetValue(&appTimer_C, LORAWAN_APP_INTERVAL_C);

    TimerStart(&appTimer_A);
    TimerStart(&appTimer_B);
    TimerStart(&appTimer_C);
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
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = gAppPort;
  lmh_send(&m_lora_app_data, g_CurrentConfirm);
}

/* =============================================================================== */

/**@brief Function for sending the Temperatur and Humidity data
 */
void sendTemperaturandHumidityData()
{
  /* restart teh timer */
  TimerSetValue(&appTimer_A, LORAWAN_APP_INTERVAL_A);
  TimerStart(&appTimer_A);  
  Serial.println("Sending Temperatur and humidity frame now...");
  Serial.println("try to take the lora semaphore");
  xSemaphoreTake(loraSend, portMAX_DELAY);

  /* get Temp + Humidity Values */
  String data = "";
  float temp = 0;
  float hum = 0;

  /* update SHT3 values */
  if (SHTC3_Status_Nominal == mySHTC3.update())
  {
    /* get the updated values */
    temp = mySHTC3.toDegC();
    hum = mySHTC3.toPercent();
  }
  
  /* print SHT3 values */
  data = "Tem:" + String(temp) + "C " + "Hum:" + String(hum) + "% ";
  Serial.println(data);

  uint16_t t = (uint16_t)(temp * 100);
  uint16_t h = (uint16_t)(hum * 100);

  /* prepare lora buffer */
  uint32_t i = 0;
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;
  m_lora_app_data.buffer[i++] = DEVICE_NUMBER; // indicate the Device
  m_lora_app_data.buffer[i++] = 0x01; // indicate soil moisture values
  m_lora_app_data.buffer[i++] = (uint8_t)(t >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)t;
  m_lora_app_data.buffer[i++] = (uint8_t)(h >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)h;
  m_lora_app_data.buffsize = i;

  /* send data */
  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS)
  {
    count++;
    Serial.printf("lmh_send ok count %d\n", count);
  }
  else
  {
    count_fail++;
    Serial.printf("lmh_send fail count %d\n", count_fail);
  }
  
  Serial.println("...sended");
  /* wait before releasing the semaphore to ensure Lora proper communicaion */
  Serial.println("wait a bit");
  delay(FAIRUSE_TTN_INTERVAL);
  Serial.println("give the lora semaphore");
  xSemaphoreGive(loraSend);
}
void sendRainData()
{
  /* restart teh timer */
  TimerSetValue(&appTimer_B, LORAWAN_APP_INTERVAL_B);
  TimerStart(&appTimer_B);  
  Serial.println("Sending Raindata frame now...");
  Serial.println("try to take the lora semaphore");
  xSemaphoreTake(loraSend, portMAX_DELAY);

  /* get Temp + Humidity Values */
  String data = "";
  float acc = 0;
  float eventAcc = 0;
  float totalAcc = 0;

  /* update RG-15 values */
  String stracc;
  String streventacc;
  String strtotalacc;    
  String Snapshot = rg15.getAvailable(); //gets and clears the stream buffer string.
  stracc = Snapshot.substring(3,9);
  acc = stracc.toFloat();
  streventacc = Snapshot.substring(22,28);
  eventAcc = streventacc.toFloat();
  strtotalacc = Snapshot.substring(41,47);
  totalAcc = strtotalacc.toFloat();
//  Serial.println(Snapshot);
//  Serial.println("-------------------------------------------");
  
  /* print SHT3 values */
  data = "Acc:" + String(acc) + "mm " + "eventAcc:" + String(eventAcc) + "mm " + "totalAcc:" + String(totalAcc) + "mm ";
  Serial.println(data);

  uint16_t a = (uint16_t)(acc * 100);
  uint16_t ea = (uint16_t)(eventAcc * 100);
  uint16_t ta = (uint16_t)(totalAcc * 100);

  /* prepare lora buffer */
  uint32_t i = 0;
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;
  m_lora_app_data.buffer[i++] = DEVICE_NUMBER; // indicate the Device
  m_lora_app_data.buffer[i++] = 0x02; // indicate soil moisture values
  m_lora_app_data.buffer[i++] = (uint8_t)(a >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)a;
  m_lora_app_data.buffer[i++] = (uint8_t)(ea >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)ea;
  m_lora_app_data.buffer[i++] = (uint8_t)(ta >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)ta;
  m_lora_app_data.buffsize = i;

  /* send data */
  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS)
  {
    count++;
    Serial.printf("lmh_send ok count %d\n", count);
  }
  else
  {
    count_fail++;
    Serial.printf("lmh_send fail count %d\n", count_fail);
  }
  
  Serial.println("...sended");
  /* wait before releasing the semaphore to ensure Lora proper communicaion */
  Serial.println("wait a bit");
  delay(FAIRUSE_TTN_INTERVAL);
  Serial.println("give the lora semaphore");
  xSemaphoreGive(loraSend);
}

/**@brief Function for sending the GPS data
 */
void sendBatterieData()
{
  /* restart teh timer */
  TimerSetValue(&appTimer_C, LORAWAN_APP_INTERVAL_C);
  TimerStart(&appTimer_C);  
  Serial.println("Sending battery frame now...");
  Serial.println("try to take the lora semaphore");
  xSemaphoreTake(loraSend, portMAX_DELAY);


  // Get a raw ADC reading
  float f32_vbat_mv = readVBAT();
  // Convert from raw mv to percentage (based on LIPO chemistry)
  uint8_t u8_vbat_per = mvToPercent(f32_vbat_mv);
  // get battery value
  uint8_t u8_bat = mvToLoRaWanBattVal(f32_vbat_mv);
  Serial.println("u8_vbat_per: " + String(u8_vbat_per));
  
  /* prepare lora buffer */
  uint32_t i = 0;
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;
  m_lora_app_data.buffer[i++] = DEVICE_NUMBER; // indicate the Device
  m_lora_app_data.buffer[i++] = 0x03; // indicate battery values
  m_lora_app_data.buffer[i++] = u8_vbat_per;
  m_lora_app_data.buffsize = i;

  /* send data */
  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS)
  {
    count++;
    Serial.printf("lmh_send ok count %d\n", count);
  }
  else
  {
    count_fail++;
    Serial.printf("lmh_send fail count %d\n", count_fail);
  }
  
  Serial.println("...sended");
  /* wait before releasing the semaphore to ensure Lora proper communicaion */
  Serial.println("wait a bit");
  delay(FAIRUSE_TTN_INTERVAL);
  Serial.println("give the lora semaphore");
  xSemaphoreGiveFromISR(loraSend, pdFALSE);
}
/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
uint32_t timers_init(void)
{
  TimerInit(&appTimer_A, sendTemperaturandHumidityData);
  TimerInit(&appTimer_B, sendRainData);
  TimerInit(&appTimer_C, sendBatterieData);
  return 0;
}
