/********************************************
* configuration file for LoRa_WX-TTGO_TK5EP *
* dec 2023                                 *
********************************************/

/***************************************************DEBUG_BME
  DEBUG flags comment out (remove "//") what is needed to be displayed on terminal
***************************************************/
//#define DEBUG_WINDSPEED
//#define DEBUG_RAIN
//#define DEBUG_WIND
//#define DEBUG_BME
//#define DEBUG_SHT
//#define DEBUG_TIME
//#define DEBUG_WG
//#define DEBUG_RS485
//#define DEBUG_MQTT

/***************************************************
  HARDWARE comment out (remove "//") the followings lines depending of sensor used.
***************************************************/
#define WITH_BME280
//#define WITH_SHT31
//#define WITH_RAIN                             // comment if NO rain sensor
#define WITH_WIND								         // comment if NO wind sensor

/***************************************************
   Protocols used
***************************************************/
//#define WITH_APRS_LORA                          // use APRS LoRa or not
#define WITH_WIFI                               // needed for MQTT, WUNDERGROUND, APRS-IS and WEBPAGE
#define WITH_WUNDERGROUND                       // send WX repots to Wunderground
//#define WITH_MQTT								         // use MQTT broker or not
#define WITH_APRS_IS							         // use APRS-IS instead of APRS via LoRa

// if no WiFi is set, unselect all protocols that need WiFi
#ifndef WITH_WIFI
   #undef WITH_WUNDERGROUND
   #undef WITH_MQTT
   #undef WITH APRS_IS
#endif

/***************************************************
   OLED display
***************************************************/
const byte ECOMODE               = 0;             	   // screensaver 0=OFF 1=PERMANENT 2=ONLY BETWEEN TX
const bool DISPLAY_CARDINAL      = false;             // display directions on OLED. true = cardinal. false = degrees

/***************************************************
   Station coordinates
***************************************************/
#define CALLSIGN "NOCALL-13"                     // callsign with SSID ex TK5EP-13
#define LATITUDE "0000.00N"                     // latitude in DDMM.MM NORTH ex : 4156.95N
#define LONGITUDE "00000.00E"                   // longitude in DDDMM.MM EAST ex: 00845.26E
const uint16_t ALTITUDE = 100;                   // home altitude in meters
#define WITH_SEALEVELPRESSURE                   // if pressure on sealevel has to be used

/***************************************************
   Sensors
***************************************************/
const uint8_t BME280_I2C         = 0x76;              // BME280 i2c address either 0x76 or 0x77
const uint8_t SHT31_I2C          = 0x44;              // SHT31 i2c address either 0x44 or 0x45
const float rainBucketCont       = 0.28;              // rain sensor bucket containance in mm of water, depends on model
const byte RAINDEBOUNCE          = 250;               // in ms debounce time for the rain sensor

// ModBus wind sensors
const uint8_t NewSensorAddress   = 0x02;              // put here the new sensor address in case of needed change. Certainly one of two addresses below. Be sure to have only one sensor on ModBus. 
const uint8_t AddressSpeedSensor = 0x01;      		   // wind speed sensor ModBus address
const uint8_t AddressDirSensor   = 0x02;        		// wind direction sensor ModBus address

/***************************************************
    LoRa
***************************************************/
const long TXFREQUENCY           = 433775000;         // TX frequency in Hz
const int TXPERIOD               = 120;               // TX period in seconds
const int TXPOWER                = 20;                // power in dBm
#define COMMENT                  " "          		   // short info in beacon. Leave blank if not wanted
#define WITH_DIGIPEATING                              // if we want the APRS frames to be repeated

/***************************************************
   WIFI
***************************************************/
#define ssid              "SSID"                     // Change this to your WiFi SSID
#define password          "PASSWORD"      // Change this to your WiFi password
const bool WITH_STATIC_IP = false;			  		      // true=static address or false=DHCP
const IPAddress local_IP(44, 168, 80, 141);		      // Set your Static IP address like xxx,xxx,xxx,xxx
const IPAddress gateway(44, 168, 80, 129);		      // Set your Gateway IP address
const IPAddress subnet(255, 255, 255, 240);			   // Set your subnet mask
const IPAddress primaryDNS(44, 168, 80, 129);		  	// optional, put 8.8.8.8 if you don't know
//IPAddress secondaryDNS(8, 8, 4, 4); 		  	         // optional

/***************************************************
   WUNDERGROUND
***************************************************/
#define WG_server "weatherstation.wunderground.com"   // Wunderground server address
#define WG_ID     "WG_ID"                          // Your WunderGround ID. You need to register to get this ID and Key
#define WG_PWD    "WG_KEY"                          // Your WunderGround key

/***************************************************
   MQTT Broker
***************************************************/
#define mqtt_broker          "test.mosquitto.org"// the MQTT server
const int mqtt_port           = 1883;					    // MQTT server port 1883 by default
const byte mqtt_retained      = 1;                     // should the datas be retained ?
#define mqtt_username         ""							    // MQTT server username if needed, leave blank if not
#define mqtt_password         ""							    // MQTT server password if needed, leave blank if not
#define TOPIC_TEMP            "wx/temp"			    // topic for the temperature
#define TOPIC_HUMI            "wx/humi"			    // topic for the humidity
#define TOPIC_PRESS           "wx/press"			    // topic for the pressure
#define TOPIC_WINDSPEED       "wx/windspeed"	    // topic for the wind speed
#define TOPIC_WINDDIR         "wx/winddir"		    // topic for the wind direction
#define TOPIC_GUSTSPEED       "wx/gustspeed"	    // topic for the wind speed
#define TOPIC_GUSTDIR         "wx/gustdir"		    // topic for the wind direction
#define TOPIC_RAIN            "wx/rain"			    // topic for the rain fall
#define TOPIC_RAIN24H         "wx/rain24h"		    // topic for the 24 h rain fall
#define TOPIC_WINDSPEEDSENSOR "wx/windspeedsensor" // topic to check if the RS485 dialog is OK
#define TOPIC_WINDDIRSENSOR   "wx/winddirsensor"   // topic to check if the RS485 dialog is OK

/***************************************************
   APRS-IS
***************************************************/
#define APRS_IS_CALLSIGN "NOCALL-13"					      // APRS IS callsign with SSID, normally -13
#define APRS_IS_PASSWD   "11111"						      // APRS IS password, get it at https://apps.magicbug.co.uk/passcode/
#define APRS_IS_SERVER   "rotate.aprs.net" // APRS server address, ie rotate.aprs.net
const int APRS_IS_SERVER_PORT = 14580;						// APRS server port, default 14580

/***************************************************
   OTA
***************************************************/
#define OTA_username       "root"                 // OTA access login
#define OTA_password       "password"                // OTA access password

/***************************************************
   NTP
***************************************************/
// Adjust your own dailight saving time parameters here
// http://www.timezoneconverter.com/cgi-bin/zoneinfo?s=standard&tz=Europe/Paris
// ntp.ruleDST("CEST", Last, Sun, Mar, 2, 120); // last sunday in march 2:00, timetone +120min (+1 GMT + 1h summertime offset)
#define      DSTzone       "CEST"
const int8_t DSTweek       = Last;
const int8_t DSTwday       = Sun;
const int8_t DSTmonth      = Mar;
const int8_t DSTday        = 2;
const int8_t DSToffset     = 120;
//ntp.ruleSTD("CET", Last, Sun, Oct, 3, 60); // last sunday in october 3:00, timezone +60min (+1 GMT)
#define      STDzone       "CET"
const int8_t STDweek       = Last;
const int8_t STDwday       = Sun;
const int8_t STDmonth      = Oct;
const int8_t STDday        = 3;
const int8_t STDoffset     = 60;
