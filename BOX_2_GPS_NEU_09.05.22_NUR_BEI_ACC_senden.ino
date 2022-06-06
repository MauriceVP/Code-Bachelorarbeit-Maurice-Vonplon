  /**
 * @file GPS_Tracker.ino
 * @author rakwireless.com
 * @brief This sketch demonstrate a GPS tracker that collect location from a uBlox M7 GNSS sensor
 *    and send the data to lora gateway.
 *    It uses a 3-axis acceleration sensor to detect movement of the tracker
 * @version 0.1
 * @date 2020-07-28
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
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <LoRaWan-RAK4630.h> //http://librarymanager/All#SX126x
#include <SPI.h>
#include "SparkFunLIS3DH.h" //http://librarymanager/All#SparkFun-LIS3DH
#include "Wire.h"
#include <TinyGPS.h>        //http://librarymanager/All#TinyGPS


/* Acceleration stuff */
LIS3DH SensorAcc(I2C_MODE, 0x18);

/* GPS stuff */
TinyGPS gps;
String tmp_data = "";
int direction_S_N = 0;  //0--S, 1--N
int direction_E_W = 0;  //0--E, 1--W

// RAK4630 supply two LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

/* battery stuff */
#define PIN_VBAT WB_A0
#define PIN_HUM  WB_A1

uint32_t humidity_pin = PIN_HUM;
uint32_t vbat_pin = PIN_VBAT;

#define VBAT_MV_PER_LSB (0.73242188F) // 3.0V ADC range and 12 - bit ADC resolution = 3000mV / 4096
#define VBAT_DIVIDER_COMP (1.73)      // Compensation factor for the VBAT divider, depend on the board

#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)


/* LoRa stuff */
bool doOTAA = true;   // OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60										  /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_0									  /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5								  /*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 3										  /**< Number of trials for the join request. */
DeviceClass_t gCurrentClass = CLASS_A;							  /* class definition*/
LoRaMacRegion_t gCurrentRegion = LORAMAC_REGION_EU868;    /* Region:EU868*/
lmh_confirm gCurrentConfirm = LMH_CONFIRMED_MSG;				  /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;							  /* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
 */
static lmh_param_t lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);

static float readVBAT(void);
static uint8_t mvToPercent(float mvolts);
static uint8_t mvToLoRaWanBattVal(float mvolts);
static void direction_parse(String tmp);
static bool getGpsData(float* f32_flat, float* f32_flon, uint8_t* u8_lora_satellites);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                        lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler, lorawan_join_failed_handler
                                       };

//OTAA keys !!!! KEYS ARE MSB !!!!
uint8_t nodeDeviceEUI[8] = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x06, 0x7A, 0x11};
uint8_t nodeAppEUI[8] = {0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10};
uint8_t nodeAppKey[16] = {0x22, 0x8C, 0x54, 0x82, 0x0C, 0x0B, 0xC7, 0x7C, 0x03, 0xF3, 0xC2, 0x44, 0xE5, 0x3B, 0x0F, 0x07};

// ABP keys
uint32_t nodeDevAddr = 0x260116F8;
uint8_t nodeNwsKey[16] = {0x7E, 0xAC, 0xE2, 0x55, 0xB8, 0xA5, 0xE2, 0x69, 0x91, 0x51, 0x96, 0x06, 0x47, 0x56, 0x9D, 0x23};
uint8_t nodeAppsKey[16] = {0xFB, 0xAC, 0xB6, 0x47, 0xF3, 0x58, 0x45, 0xC7, 0x50, 0x7D, 0xBF, 0x16, 0x8B, 0xA8, 0xC1, 0x7C};

// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE            64										      /**< buffer size of the data to be transmitted. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];			  //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.
static uint32_t count = 0;
static uint32_t count_fail = 0;

/* timer stuff */
#define FAIRUSE_TTN_INTERVAL      1000*1        /* with TTN dont go below 35s send interval (for payed services it will be ok to send faster) */
#define LORAWAN_APP_INTERVAL_A    1000*60*30    /* A timer: 1000ms * x (Bodenfeuchtigkeit) Every 30min  */
#define LORAWAN_APP_INTERVAL_B    1000*60*13    /* C timer: 1000ms * x (Batterie)          Every 13min  */
#define LOOP_INTERVAL             1000*10
#define DEVICE_NUMBER             0x02




TimerEvent_t appTimer_A;
TimerEvent_t appTimer_B;

static uint32_t timers_init(void);

// Globale Variable Beschleunigungssensor
float Global_X;
float Global_Y;
float Global_Z;

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


/* = GPS ================================================================================================================================= */

/**@brief checking direction on earth.
 */
void direction_parse(String tmp)
{
    if (tmp.indexOf(",E,") != -1)
    {
        direction_E_W = 0;
    }
    else
{
        direction_E_W = 1;
    }

    if (tmp.indexOf(",S,") != -1)
    {
        direction_S_N = 0;
    }
    else
    {
        direction_S_N = 1;
    }
}


bool getGpsData(float* pf32_flat, float* pf32_flon, uint8_t* pu8_lora_satellites)
{
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, 1);
  Serial.println("GPS-Modul is ON");

  bool retVal = false;

    /* GPS stuff */
  bool newData = false;
  unsigned long chars, runtimeInS = 0;
  unsigned short sentences, failed;
  Serial.println("lets try to get GPS data");
  /* we loop  */
  while((newData == false) && (runtimeInS < 120))
  {
    //For one second we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
      while (Serial1.available())
      {
        char c = Serial1.read();
        tmp_data += c;
        if (gps.encode(c))
        {
           newData = true;
           Serial.println("got data");
        }
      }
      delay(5);
    }
    /* one second was gone */
    runtimeInS++;
    Serial.println("GPS loop runtimeInS: " + String(runtimeInS));
  }
  /* check dirction */
  direction_parse(tmp_data);
  
  tmp_data = "";
  Serial.println("gps polling done");
  
  if (newData)
  {
    /* we have data */
    retVal = true;
    Serial.println("gps data received");
    
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    if(direction_S_N == 0)
    {
      Serial.print("(S):");
    }
    else
    {
      Serial.print("(N):");
    }
    Serial.print("Längengrad=");
    //Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    if(flat != TinyGPS::GPS_INVALID_F_ANGLE)
    {
       *pf32_flat = flat;
    }
    if(direction_E_W == 0)
    {
      Serial.print(" (O):");
    }
    else
    {
      Serial.print(" (W):");
    }
    Serial.print("Breitengrad=");
    //Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);

    if(flon != TinyGPS::GPS_INVALID_F_ANGLE)
    {
       *pf32_flon = flon;
    }

    Serial.print(" Satelliten=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());

    if(gps.satellites() != TinyGPS::GPS_INVALID_SATELLITES)
    {
       *pu8_lora_satellites = gps.satellites();
    }
    
    Serial.print(" Genauigkeit=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
  
  
  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
  {
    Serial.println("** No characters received from GPS: check wiring **");
  }

  digitalWrite(WB_IO2, 0);
  Serial.println("GPS-Modul is OFF");

  retVal = true;
  return retVal; 
}


void checkAndSendAcc()
{
  /* set the threshold value */
  float threshold = 0.05;

  Serial.println("Check ACC values");
  /* obtain  */
  float New_X = SensorAcc.readFloatAccelX();
  float New_Y = SensorAcc.readFloatAccelY();
  float New_Z = SensorAcc.readFloatAccelZ();

  Serial.println("Global_X: " + String(Global_X) + " New_X" + String(New_X));
  Serial.println("Global_Y: " + String(Global_Y) + " New_Y" + String(New_Y));
  Serial.println("Global_Y: " + String(Global_Z) + " New_Z" + String(New_Z));

  if((abs(Global_X - New_X) > threshold) || (abs(Global_Y - New_Y) > threshold) || (abs(Global_Z - New_Z) > threshold))
  {
    Global_X = New_X;
    Global_Y = New_Y;
    Global_Z = New_Z;

    /* Send ACC Data */
    
    Serial.println("Sending acc values now => sensor moved!");
    Serial.println("try to take the lora semaphore");
    xSemaphoreTake(loraSend, portMAX_DELAY);
    
    /* prepare lora buffer */
    int16_t u16_x = (int16_t)(Global_X*1000)+1000;
    int16_t u16_y = (int16_t)(Global_Y*1000)+1000;
    int16_t u16_z = (int16_t)(Global_Z*1000)+1000;
    
    uint32_t i = 0;
    memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
    m_lora_app_data.port = gAppPort;
    m_lora_app_data.buffer[i++] = DEVICE_NUMBER; // indicate the Device
    m_lora_app_data.buffer[i++] = 0x04; // indicate Accel values
    m_lora_app_data.buffer[i++] = (uint8_t)(u16_x >> 8);
    m_lora_app_data.buffer[i++] = (uint8_t)u16_x;
    m_lora_app_data.buffer[i++] = (uint8_t)(u16_y >> 8);
    m_lora_app_data.buffer[i++] = (uint8_t)u16_y;
    m_lora_app_data.buffer[i++] = (uint8_t)(u16_z >> 8);
    m_lora_app_data.buffer[i++] = (uint8_t)u16_z;
    m_lora_app_data.buffsize = i;

    lmh_error_status error = lmh_send(&m_lora_app_data, gCurrentConfirm);
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
    delay(FAIRUSE_TTN_INTERVAL);
    Serial.println("give the lora semaphore");
    xSemaphoreGiveFromISR(loraSend, pdFALSE);

        /* First Send GPS Frame */

    Serial.println("Sending GPS frame now...");
  
    float f32_flat = 0; 
    float f32_flon = 0;
    uint8_t u8_lora_satellites = 0;
    
    /* get GPS data */
    getGpsData(&f32_flat, &f32_flon, &u8_lora_satellites);
  
    if (u8_lora_satellites != 0)
    {
      delay(1000*10);
      
      Serial.println("try to take the lora semaphore");
      xSemaphoreTake(loraSend, portMAX_DELAY);
      
      uint32_t u32_lat = ((uint32_t)(f32_flat*10000000));
      uint32_t u32_lon = ((uint32_t)(f32_flon*10000000));
    
      /* prepare lora buffer */
      uint32_t i = 0;
      memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
      m_lora_app_data.port = gAppPort;
      m_lora_app_data.buffer[i++] = DEVICE_NUMBER; // indicate the Device
      m_lora_app_data.buffer[i++] = 0x05; // indicate GPS values
      m_lora_app_data.buffer[i++] = u8_lora_satellites;
      m_lora_app_data.buffer[i++] = (uint8_t)(u32_lat >> 24);
      m_lora_app_data.buffer[i++] = (uint8_t)(u32_lat >> 16);
      m_lora_app_data.buffer[i++] = (uint8_t)(u32_lat >> 8);
      m_lora_app_data.buffer[i++] = (uint8_t)u32_lat;
      m_lora_app_data.buffer[i++] = (uint8_t)(u32_lon >> 24);
      m_lora_app_data.buffer[i++] = (uint8_t)(u32_lon >> 16);
      m_lora_app_data.buffer[i++] = (uint8_t)(u32_lon >> 8);
      m_lora_app_data.buffer[i++] = (uint8_t)u32_lon;
      m_lora_app_data.buffsize = i;
  
      /* send data */
      lmh_error_status error = lmh_send(&m_lora_app_data, gCurrentConfirm);
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
  }
  else
  {
    Serial.println("acc values not out of range => sensor was not moved");
    Global_X = New_X;
    Global_Y = New_Y;
    Global_Z = New_Z;
  }
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

  switch (gCurrentRegion)
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

	//lis3dh init
	if (SensorAcc.begin() != 0)
	{
		Serial.println("Problem starting the sensor at 0x18.");
	}
	else
	{
		Serial.println("Sensor at 0x18 started.");
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
  err_code = lmh_init(&lora_callbacks, lora_param_init, doOTAA, gCurrentClass, gCurrentRegion);
	if (err_code != 0)
	{
		Serial.printf("lmh_init failed - %d\n", err_code);
    return;
	}

	// Start Join procedure
	lmh_join();

//  pinMode(WB_A0, INPUT_PULLDOWN);
//  pinMode(WB_A1, INPUT_PULLDOWN);

  Serial.println("RAK4631 Battery checker init");
  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);
  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14
  // Let the ADC settle
  delay(1);
  // Get a single ADC sample and throw it away
  readVBAT();
  

  analogOversampling(128);


  //gps off - on
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, 0);
  delay(1000);
  digitalWrite(WB_IO2, 1);
  delay(1000);

  Serial1.begin(9600);
  while (!Serial1);
  {
      delay(1);
  }
  Serial.println("GPS uart init ok!");
  
  /* create a semaphore */
  loopEnable = xSemaphoreCreateBinary();
  xSemaphoreGive(loopEnable);
  xSemaphoreTake(loopEnable, (TickType_t)10);

  loraSend = xSemaphoreCreateBinary();
  xSemaphoreGive(loraSend);

  Global_X = 0;
  Global_Y = 0;
  Global_Z = 0; 
  
}



void loop()
{
  // Put your application tasks here, like reading of sensors,
  // Controlling actuators and/or other functions. 
  
  Serial.println("RAK4631:- I am going for a Nap (waiting for Semaphore)");

//  digitalWrite(LED_BUILTIN2, LOW);  //turn the LED on (HIGH is the voltage level)
  /* try to get the Semaphore */
  xSemaphoreTake(loopEnable, (TickType_t)LOOP_INTERVAL);
  /* woke up - show LED */
//  digitalWrite(LED_BUILTIN2, HIGH); //turn the LED off by making the voltage LOW
  delay(500);                       // wait for a second
  Serial.println("RAK4631:- Hey I just Woke up (got Semaphore)");
  /* check if senors was moved */
  checkAndSendAcc();
}


/* ============================================================================================================================================== */


/**@brief LoRa function for handling HasJoined event.
 */
void lorawan_has_joined_handler(void)
{
	Serial.println("OTAA Mode, Network Joined!");

	lmh_error_status ret = lmh_class_request(gCurrentClass);
	if (ret == LMH_SUCCESS)
	{
		delay(1000);
		TimerSetValue(&appTimer_A, LORAWAN_APP_INTERVAL_A);
    TimerSetValue(&appTimer_B, LORAWAN_APP_INTERVAL_B);

		TimerStart(&appTimer_A);
    TimerStart(&appTimer_B);
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
	lmh_send(&m_lora_app_data, gCurrentConfirm);
}

/* =============================================================================== */

/**@brief Function for sending the soil moisture data
 */
void sendSoilMoisture()
{
  /* restart teh timer */
  TimerSetValue(&appTimer_A, LORAWAN_APP_INTERVAL_A);
  TimerStart(&appTimer_A);  
  Serial.println("Sending soil moisture frame now...");
  Serial.println("try to take the lora semaphore");
  xSemaphoreTake(loraSend, portMAX_DELAY);

  /* get Soilmoisture Values */
  String data = "";
  float m = 0;
  float Bodenf = 0;
  uint32_t bf = 0;
  
  m = analogRead(humidity_pin);
  data = String(m);
  Serial.println(data);

  Bodenf = (m*0.505*100*3)/4096; //Umrechnung für den Bodenfeuchtigkeitssensor
  delay(1000);
  
  /* print Soil values */
  data = "Bodenfeuchtigkeit:" + String(Bodenf) + "%";
  Serial.println(data);

  bf = ((uint32_t)(Bodenf*100));
 
  
  data = String(bf);
  Serial.println(data);

  /* prepare lora buffer */
  uint32_t i = 0;
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;
  m_lora_app_data.buffer[i++] = DEVICE_NUMBER; // indicate the Device
  m_lora_app_data.buffer[i++] = 0x06; // indicate soil moisture values
  m_lora_app_data.buffer[i++] = (uint8_t)(bf >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)bf;
  m_lora_app_data.buffsize = i;

  /* send data */
  lmh_error_status error = lmh_send(&m_lora_app_data, gCurrentConfirm);
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

/**@brief Function for sending the Batterie data
 */
void sendBatterieData()
{
  /* restart teh timer */
  TimerSetValue(&appTimer_B, LORAWAN_APP_INTERVAL_B);
  TimerStart(&appTimer_B);  
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
  lmh_error_status error = lmh_send(&m_lora_app_data, gCurrentConfirm);
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
	TimerInit(&appTimer_A, sendSoilMoisture);
  TimerInit(&appTimer_B, sendBatterieData);
	return 0;
}
