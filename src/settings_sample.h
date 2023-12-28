/********************************************
* configuration file for LoRa_WX-TTGO_TK5EP *
* dec 2023                                 *
********************************************/

/***************************************************DEBUG_BME
   DEBUG flags comment out what is needed to be displayed on terminal
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
  HARDWARE comment out the followings lines depending of sensor used. The BME280 is always in use
***************************************************/
#define WITH_BME280
//#define WITH_SHT31
//#define WITH_RAIN                            // comment out if NO rain sensor
#define WITH_WIND								      // comment out if NO wind sensor

/***************************************************
   Protocols used
***************************************************/
#define WITH_APRS_LORA                          // use APRS LoRa or not
#define WITH_WIFI                               // needed for MQTT, WUNDERGROUND, APRS-IS and WEBPAGE
#define WITH_WUNDERGROUND                       // send WX repots to Wunderground
#define WITH_MQTT								         // use MQTT broker or not
//#define WITH_APRS_IS							         // use APRS-IS instead of APRS via LoRa

// if no WiFi is set, unselect all protocols that need WiFi
#ifndef WITH_WIFI
   #undef WITH_WUNDERGROUND
   #undef WITH_MQTT
   #undef WITH APRS_IS
#endif

/***************************************************
   OLED display
***************************************************/
byte ECOMODE               = 0;             	   // screensaver 0=OFF 1=PERMANENT 2=ONLY BETWEEN TX
bool DISPLAY_CARDINAL      = false;             // display directions on OLED. true = cardinal. false = degrees

/***************************************************
   Station coordinates
***************************************************/
// PUNTA
#define CALLSIGN           "NOCALL-13"           // callsign with SSID ex TK5EP-10
#define LATITUDE           "0000.00N"           // latitude in DDMM.MM NORTH ex : 4156.95N
#define LONGITUDE          "00000.00E"          // longitude in DDDMM.MM EAST ex: 00845.26E
uint16_t ALTITUDE          = 760;               // home altitude in meters
#define WITH_SEALEVELPRESSURE			// if pressure on sealevel has to be used

/***************************************************
   Sensors
***************************************************/
uint8_t BME280_I2C         = 0x76;              // BME280 i2c address either 0x76 or 0x77
uint8_t SHT31_I2C          = 0x44;              // SHT31 i2c address either 0x44 or 0x45
const float rainBucketCont = 0.28;              // rain sensor bucket containance in mm of water, depends on model
const byte RAINDEBOUNCE    = 250;               // in ms debounce time for the rain sensor

// ModBus wind sensors
uint8_t NewSensorAddress   = 0x02;              // put here the new sensor address in case of needed change. Certainly one of two addresses below. Be sure to have only one sensor on ModBus. 
uint8_t AddressSpeedSensor = 0x01;      		   // wind speed sensor ModBus address
uint8_t AddressDirSensor   = 0x02;        		// wind direction sensor ModBus address

/***************************************************
    LoRa
***************************************************/
long TXFREQUENCY           = 433775000;         // TX frequency in Hz
int TXPERIOD               = 120;               // TX period in seconds
int TXPOWER                = 20;                // power in dBm
#define COMMENT            " "          		   // short info in beacon. Leave blank if not wanted
//#define WITH_DIGIPEATING                        // if we want the APRS frames to be repeated

/***************************************************
   WIFI
***************************************************/
const char* ssid =              "SSID";                 // Change this to your WiFi SSID
const char* password =          "AP-PASSWORD";  // Change this to your WiFi password
bool WITH_STATIC_IP = false;			  			            // true=static address or false=DHCP
IPAddress local_IP(44, 168, 80, 141);		  	            // Set your Static IP address like xxx,xxx,xxx,xxx
IPAddress gateway(44, 168, 80, 129);		  	            // Set your Gateway IP address
IPAddress subnet(255, 255, 255, 240);			            // Set your subnet mask
IPAddress primaryDNS(44, 168, 80, 129);		  	         // optional, put 8.8.8.8 if you don't know
//IPAddress secondaryDNS(8, 8, 4, 4); 		  	              // optional

/***************************************************
   WUNDERGROUND
***************************************************/
const char* WG_server =    "weatherstation.wunderground.com";   // Wunderground server address
const char* WG_ID =        "IWG_ID";                // Your WunderGround ID. You need to register to get this ID and Key
const char* WG_PWD =       "WG_KEY";                 // Your WunderGround key

/***************************************************
   MQTT Broker
***************************************************/
const char *mqtt_broker = "test.mosquitto.org";
const int mqtt_port        = 1883;								// MQTT server port 1883 by default
const byte mqtt_retained   = 1;                          // should the datas be retained ?
const char* mqtt_username  = "";								   // MQTT server username if needed, leave blank if not
const char* mqtt_password  = "";								   // MQTT server password if needed, leave blank if not

// PUNTA
const char* TOPIC_TEMP     = "wx/temp";				// topic for the temperature
const char* TOPIC_HUMI     = "wx/humi";				// topic for the humidity
const char* TOPIC_PRESS    = "wx/press";				// topic for the pressure
const char* TOPIC_WINDSPEED= "wx/windspeed";		   // topic for the wind speed
const char* TOPIC_WINDDIR  = "wx/winddir";			// topic for the wind direction
const char* TOPIC_GUSTSPEED= "wx/gustspeed";		   // topic for the wind speed
const char* TOPIC_GUSTDIR  = "wx/gustdir";			// topic for the wind direction
const char* TOPIC_RAIN     = "wx/rain";				// topic for the rain fall
const char* TOPIC_RAIN24H  = "wx/rain24h";			// topic for the 24 h rain fall

/***************************************************
   APRS-IS
***************************************************/
const char *APRS_IS_CALLSIGN  = "NOCALL-13";					// APRS IS callsign with SSID, normally -13
const char *APRS_IS_PASSWD    = "11111";						// APRS IS password, get it at https://apps.magicbug.co.uk/passcode/
const char *APRS_IS_SERVER    = "rotate.aprs.net"; // APRS server address, ie rotate.aprs.net
const int APRS_IS_SERVER_PORT = 14580;						   // APRS server port, default 14580

/***************************************************
   OTA
***************************************************/
const char* OTA_username      = "root";                 // OTA access login
const char* OTA_password      = "password";                // OTA access password

/***************************************************
   NTP
***************************************************/
//http://www.timezoneconverter.com/cgi-bin/zoneinfo?s=standard&tz=Europe/Paris
//const char* DSTime = "'CEST', Last, Sun, Mar, 2, 120"; // last sunday in march 2:00, timetone +120min (+1 GMT + 1h summertime offset)
//ntp.ruleSTD("CET", Last, Sun, Oct, 3, 60); // last sunday in october 3:00, timezone +60min (+1 GMT)
