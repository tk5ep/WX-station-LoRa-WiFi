/********************************************
  configuration file for LoRa_WX-TTGO_TK5EP *
  2024-12-18
********************************************/

#include <Arduino.h>

/***************************************************
   Board used
***************************************************/
// this is defined in the platformIO.ini

/***************************************************DEBUG_BME
  DEBUG flags comment out (remove "//") what is needed to be displayed on terminal
***************************************************/
//#define DEBUG_GLOBAL
//#define DEBUG_WIFI
//#define DEBUG_RAIN    // debug the rain function
//#define DEBUG_WIND    // debug the win function
//#define DEBUG_BME     // debug the BME sensors
//#define DEBUG_SHT     // debug the SHT31 sensor
//#define DEBUG_TIME    // debug the time routine
//#define DEBUG_WG      // debug the Wunderground routine
//#define DEBUG_I2C     // launch i2C scanner at startup
//#define DEBUG_NTP		// debug the NTP routine
//#define DEBUG_BAT     // debug the battery routine
//#define DEBUG_I2C     // launch i2C scanner at startup

/***************************************************
   Display
***************************************************/
//#define HAS_SH110X                              // remove "//"" if using a SH110X OLED display instead of SSD1306
const byte ECOMODE               = 0;           // screensaver 0=OFF 1=PERMANENT 2=ONLY BETWEEN TX
const bool DISPLAY_CARDINAL      = false;       // display directions on OLED. true = cardinal. false = degrees
const byte MEASURE_PERIOD        = 5;           // in seconds, temp/humidity sensor measurement rate (added 2024-12-18)

/***************************************************
   Protocols used
***************************************************/
//#define WITH_APRS_LORA                          // use APRS LoRa or not
#define WITH_APRS_IS						     // use APRS-IS instead of APRS via LoRa
const bool WITH_APRS_FALLBACK = false;           // set it if you want LoRa APRS to be a fallback when APRS_IS is down
#define WITH_WIFI                                // needed for MQTT, WUNDERGROUND, APRS-IS and WEBPAGE
#define WITH_WUNDERGROUND                        // send WX reports to Wunderground
#define WITH_MQTT								 / use MQTT broker or not
#define SEND_BAT_INFO                           // send battery voltage in APRS packets

/***************************************************
   Sensors
***************************************************/
//#define WITH_BME280
//#define WITH_BMP280
//#define WITH_BME680
#define WITH_SHT31
//#define WITH_RAIN                               // comment if NO rain sensor
#define WITH_WIND	                              // same for wind sensors
const float rainBucketCont             = 0.28;  // rain sensor bucket containance in mm of water, depends on model
const int RAINDEBOUNCE                 = 250;   // in ms debounce time for the rain sensor
// sensor addresses
const uint8_t SHT31_I2C                = 0x44;  // SHT31 i2c address either 0x44 or 0x45
const uint8_t BME_I2C                  = 0x76;  // BOSCH i2c address either 0x76 or 0x77
const uint8_t OldSensorAddress         = 0x01;  // old sensor address you want to change. Probably default one : 0x01
const uint8_t NewSensorAddress         = 0x02;  // put here the new sensor address in case of needed change. Certainly one of two addresses below. Be sure to have only one sensor on ModBus. 
const uint8_t AddressSpeedSensor       = 0x01;  // wind speed RS485 sensor ModBus address
const uint8_t AddressDirSensor         = 0x02;  // wind direction RS485 sensor ModBus address
const float OnBoardDividerCorrection   = 0.38;  // Difference in Volts to correct the onboard voltage value. For a prefect precision, measure the voltage on the power connector and adjust this parameter to match.

/***************************************************
   Station coordinates
***************************************************/
const String CALLSIGN                 = "NOCALL-13"; // callsign with SSID ex TK5EP-13
const float latitude                  = 41.954426;  // latitude in DD.MMMM
const float longitude                 = 8.699275;   // longitude in DD.MMMM
const uint16_t ALTITUDE               = 760;        // station altitude in meters
#define WITH_SEALEVELPRESSURE                       // if pressure reported at sealevel is wanted

/***************************************************
   WIFI
***************************************************/
#define wifi_ssid              ""                     // Change this to your WiFi SSID
#define wifi_password          ""      // Change this to your WiFi password
//#define WITH_STATIC_IP           			  		      // true=static address or false=DHCP
const IPAddress local_IP(44, 168, 80, 141);		      // Set your Static IP address like xxx,xxx,xxx,xxx
const IPAddress gateway(44, 168, 80, 129);		      // Set your Gateway IP address
const IPAddress subnet(255, 255, 255, 240);			   // Set your subnet mask
const IPAddress primaryDNS(44, 168, 80, 129);		  	// optional, put 8.8.8.8 if you don't know
//IPAddress secondaryDNS(8, 8, 4, 4); 		  	         // optional

/***************************************************
   APRS LoRa
***************************************************/
const float TXFREQUENCY                = 433.775;  // TX frequency in MHz
const int   TXPERIOD                   = 300 ;     // TX period in seconds
const int   MQTTPERIOD                 = 15;       // MQTT period in seconds
const int   TXPOWER                    = 20;       // power in dBm 20 max for boards with SX1278 and 22 for SX1268
#define     COMMENT                    ""          // short info in beacon. Leave blank if not wanted
#define     WITH_DIGIPEATING                       // if we want the APRS frames to be repeated (adds WIDE1-1)

/***************************************************
   APRS-IS
***************************************************/
#define APRS_IS_PASSWD   ""						      // APRS IS password, get it at https://apps.magicbug.co.uk/passcode/
#define APRS_IS_SERVER   "rotate.aprs.net" // APRS server address, ie rotate.aprs.net
const int APRS_IS_SERVER_PORT = 14580;						// APRS server port, default 14580

/***************************************************
   WUNDERGROUND
***************************************************/
#define WG_server "weatherstation.wunderground.com"   // Wunderground server address
#define WG_ID     ""                          // Your WunderGround ID. You need to register to get this ID and Key
#define WG_PWD    ""                          // Your WunderGround key

/***************************************************
   MQTT Broker
***************************************************/
#define mqtt_broker          ""// the MQTT server
const int mqtt_port           = 1883;					    // MQTT server port 1883 by default
const byte mqtt_retained      = 1;                     // should the datas be retained ?
#define mqtt_username         ""							    // MQTT server username if needed, leave blank if not
#define mqtt_password         ""							    // MQTT server password if needed, leave blank if not
#define TOPIC_TEMP            "wx/punta/temp"			    // topic for the temperature
#define TOPIC_HUMI            "wx/punta/humi"			    // topic for the humidity
#define TOPIC_PRESS           "wx/punta/press"			    // topic for the pressure
#define TOPIC_WINDSPEED       "wx/punta/windspeed"	    // topic for the wind speed
#define TOPIC_WINDDIR         "wx/punta/winddir"		    // topic for the wind direction
#define TOPIC_GUSTSPEED       "wx/punta/gustspeed"	    // topic for the wind speed
#define TOPIC_GUSTDIR         "wx/punta/gustdir"		    // topic for the wind direction
#define TOPIC_RAIN1H          "wx/punta/rain1h"			    // topic for the rain fall
#define TOPIC_RAIN24H         "wx/punta/rain24h"		    // topic for the 24 h rain fall
#define TOPIC_WINDSPEEDSENSOR "wx/punta/windspeedsensor" // topic to check if the RS485 dialog is OK
#define TOPIC_WINDDIRSENSOR   "wx/punta/winddirsensor"   // topic to check if the RS485 dialog is OK
#define TOPIC_BATTERYVOLTAGE  "wx/punta/batteryvoltage"  // topic to monitor the battery voltage

/***************************************************
   OTA
***************************************************/
#define OTA_username       ""                 // OTA access login
#define OTA_password       ""                // OTA access password

/***************************************************
   NTP
***************************************************/
// Adjust your own daylight saving time parameters here
// http://www.timezoneconverter.com/cgi-bin/zoneinfo?s=standard&tz=Europe/Paris
// last sunday in march 2:00, timetone +120min (+1 GMT + 1h summertime offset)
#define      DSTzone       "CEST"
const int8_t DSTweek       = 0; //Last, First, Second, Third, Fourth (0 - 4)
const int8_t DSTwday       = 0; // Sun, Mon, Tue, Wed, Thu, Fri, Sat (0 - 7)
const int8_t DSTmonth      = 2; //Jan, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec (0 -11)
const int8_t DSThour       = 2; 
const int8_t DSToffset     = 120;
// last sunday in october 3:00, timezone +60min (+1 GMT)
#define      STDzone       "CET"
const int8_t STDweek       = 0; //Last, First, Second, Third, Fourth (0 - 4)
const int8_t STDwday       = 0; // Sun, Mon, Tue, Wed, Thu, Fri, Sat (0 - 7)
const int8_t STDmonth      = 10;
const int8_t STDhour       = 3;
const int8_t STDoffset     = 60;

// You normally DO NOT need to modify the lines below !
// DO IT ONLY if you know what you're doing !

/***************************************************
   Define pins
***************************************************/
#if defined(LILYGO_T3_V1_6)
// define pins for a TTGO T3 module
   const byte PIN_CHANGE_ADDRESS = 25; // connect this pin to 3V3 at boot to change RS485 address. Green LED lights to warn that it is the case
   const byte interruptPinRain   = 35; // rain sensor pin
// pins 4 & 34 are used by UART0 hardwareserial for the RS485 bus
// pins 21 & 22 for the i2C bus
#endif
#if defined(TTGO_T_Beam_V1_0)  || defined(TTGO_T_Beam_V1_2) || defined(TTGO_T_Beam_V1_0_SX1268)
   const byte PIN_CHANGE_ADDRESS = 25;
   const byte interruptPinRain   = 14;
#endif

/***************************************************
   GPS
***************************************************/
//#define WITH_GPS

/***************************************************
   UNDEFS
***************************************************/
// ONLY one transmission mode allowed !
#ifdef WITH_APRS_LORA
   #undef WITH_APRS_IS
#endif
#ifdef WITH_APRS_IS
   #undef WITH_APRS_LORA
#endif
// if no WiFi is set, unselect all protocols that need WiFi
#ifndef WITH_WIFI
   #undef WITH_WUNDERGROUND
   #undef WITH_MQTT
   #undef WITH_APRS_IS
#endif
// if T3 module, onboard OLED is SSD1306
#if defined(LILYGO_T3_V1_6)
    #undef HAS_SH110X
#endif
