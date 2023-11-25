/********************************************
* configuration file for LoRa_WX-TTGO_TK5EP *
* june 2023                                 *
********************************************/

/***************************************************
   DEBUG flags comment out what is needed to be displayed on terminal
***************************************************/
//#define DEBUG_WINDSPEED
//#define DEBUG_RAIN
//#define DEBUG_WIND
//#define DEBUG_BME
//#define DEBUG_TIME
//#define DEBUG_WG
//#define DEBUG_RS485
//#define DEBUG_MQTT

/***************************************************
  HARDWARE comment out the followings lines depending of sensor used. The BME280 is always in use
***************************************************/
//#define WITH_RAIN                            // uncomment if NO rain sensor
#define WITH_WIND								      // uncomment if NO wind sensor
bool DISPLAY_CARDINAL = false;               // display directions on OLED. true = cardinal. false = degrees

/***************************************************
   Protocols used
***************************************************/
#define WITH_WIFI                               // needed for MQTT, WUNDERGROUND, APRS-IS and WEBPAGE
#define WITH_APRS_IS							         // use APRS-IS instead of APRS via LoRa
#define WITH_WUNDERGROUND                       // send WX repots to Wunderground
//#define WITH_MQTT								         // use MQTT broker or not
//#define WITH_APRS_LORA                          // use APRS LoRa or not

/***************************************************
   OLED display
***************************************************/
byte ECOMODE = 1;                           	   // screensaver 0=OFF 1=PERMANENT 2=ONLY BETWEEN TX

/***************************************************
   Station coordinates
***************************************************/
#define CALLSIGN "TK5EP-13"                     // callsign with SSID ex TK5EP-10
#define LATITUDE "4157.27N"                     // latitude in DDMM.MM NORTH ex : 4156.95N
#define LONGITUDE "00841.95E"                   // longitude in DDDMM.MM EAST ex: 00845.26E
byte ALTITUDE = 60;                             // home altitude in meters

/***************************************************
   Sensors
***************************************************/
uint8_t BME280_I2C = 0x76;                      // BME280 i2c address either 0x76 or 0x77
const float rainBucketCont = 0.28;              // rain sensor bucket containance in mm of water, depends on model
const byte RAINDEBOUNCE = 250;                   // in ms debounce time for the rain sensor
//const byte WINDDEBOUNCE = 200;                 // in ms debounce time for the anemometer

// ModBus wind sensors
uint8_t NewSensorAddress = 0x02;                // put here the new sensor address in case of needed change. Certainly one of two addresses below. Be sure to have only one sensor on ModBus. 
uint8_t AddressSpeedSensor = 0x01;      		   // wind speed sensor ModBus address
uint8_t AddressDirSensor = 0x02;        		   // wind direction sensor ModBus address

/***************************************************
    LoRa
***************************************************/
long TXFREQUENCY = 433775000;                   // TX frequency in Hz
int TXPERIOD = 120;                              // TX period in seconds
int TXPOWER = 20;                               // power in dBm
#define COMMENT " "		                		   // short info in beacon. Leave blank if not wanted

/***************************************************
   WIFI
***************************************************/
const char* ssid =              "TKNET";                 // Change this to your WiFi SSID
const char* password =          "RADIOAMATEUR$CORSICA";  // Change this to your WiFi password
bool WITH_STATIC_IP = false;			  			            // true=static address or false=DHCP
IPAddress local_IP(44, 168, 80, 141);		  	            // Set your Static IP address like xxx,xxx,xxx,xxx
IPAddress gateway(44, 168, 80, 129);		  	            // Set your Gateway IP address
IPAddress subnet(255, 255, 255, 240);			            // Set your subnet mask
IPAddress primaryDNS(44, 168, 80, 129);		  	         // optional, put 8.8.8.8 if you don't know
//IPAddress secondaryDNS(8, 8, 4, 4); 		  	              // optional

/***************************************************
   WUNDERGROUND
***************************************************/
const char* WG_server =         "weatherstation.wunderground.com";   // Wunderground server address
const char* WG_ID =             "ICORSEAJ5";                         // Your WunderGround ID. You need to register to get this ID and Key
const char* WG_PWD =            "cdab4536";                          // Your WunderGround key

/***************************************************
   MQTT Broker
***************************************************/
//const char *mqtt_broker = "test.mosquitto.org";
//const char *mqtt_broker = "89.234.155.3";
const char* mqtt_broker =       "iot.radioamateur.corsica";		// the MQTT server
const int mqtt_port = 1883;									         // MQTT server port 1883 by default
const char* mqtt_username =     "";								      // MQTT server username if needed, leave blank if not
const char* mqtt_password =     "";								      // MQTT server password if needed, leave blank if not
const char* TOPIC_TEMP =        "tk5ep/qra/temp";				   // topic for the temperature
const char* TOPIC_HUMI =        "tk5ep/qra/humi";				   // topic for the humidity
const char* TOPIC_PRESS =       "tk5ep/qra/press";				   // topic for the pressure
const char* TOPIC_WINDSPEED =   "tk5ep/qra/windspeed";		   // topic for the wind speed
const char* TOPIC_WINDDIR =     "tk5ep/qra/winddir";			   // topic for the wind direction
const char* TOPIC_GUSTSPEED =   "tk5ep/qra/gustspeed";		   // topic for the wind speed
const char* TOPIC_GUSTDIR =     "tk5ep/qra/gustdir";			   // topic for the wind direction
const char* TOPIC_RAIN =        "tk5ep/qra/rain";				   // topic for the rain fall
const char* TOPIC_RAIN24H =     "tk5ep/qra/rain24h";			   // topic for the 24 h rain fall

/***************************************************
   APRS-IS
***************************************************/
const char *APRS_IS_CALLSIGN =  "TK5EP-13";					      // APRS IS callsign with SSID, normally -13
const char *APRS_IS_PASSWD =    "17132";						      // APRS IS password, get it at https://apps.magicbug.co.uk/passcode/
//const char *APRS_IS_SERVER = "rotate.aprs.net";				   // APRS server or server pool
const char *APRS_IS_SERVER =    "aprs-public.radioamateur.corsica"; // APRS server address
const int APRS_IS_SERVER_PORT = 14580;						         // APRS server port, default 14580

/***************************************************
   OTA
***************************************************/
const char* OTA_username =          "root";                 // OTA access login
const char* OTA_password =          "tk5kp";                // OTA access password