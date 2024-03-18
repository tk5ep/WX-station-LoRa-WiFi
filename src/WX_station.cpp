/***************************************************
* WX station using APRS LoRa protocol
* build for Lilygo boards
* 
Hardware :
----------
Build around a TTGO T3 LoRa ESP32 module
BME280 on i2C pins SDA=IO21, SCL=IO22
RS485 on pins RXd IO34 TXd IO0
Rain sensor on pin IO14
RS485 programming switch on IO25

Principle:
----------
sends regularily weather datas via APRS beacon on LoRa 433.775 MHz. Via WiFi to APRS-IS, Wunderground or a MQTT broker at a defined period : TXPERIOD
can use HCP or static IP address
measures temperature,pressure, humidity every TXperiod
measures rain every second and stores every min and past hour
measures wind speed, direction every second, calculates average every 2 min
measures wind gust/dir and stores every min and tracks max for past 10 min

Updates :
---------
150324 Using vector method for the average wind direction, using math.h.
290224 Using tickertwo library instead of millis() in main loop, more precise timing.
280224 RS485 address change routine rework, and small bug corrections. Added OldSensorAddress parameter in settings.h
180124 Added BMP280 support. Adapted display and Web page.
160124 Added i2C scanner, several small corrections and code cleaning.
150124 Changed lat/log format and added conversion functions.
140124 Added status packet at startup.
130124 Added battery voltage measurement.
120124 Added BME680 support
100124 Fixed several small bugs and cosmestics.
090124 Completely rewritten to uses modules. Multiple boards possible. Uses some ideas taken from CA2RXU tracker software.
311223 Added parameters in settings_sample for daylight saving time. Reworked variables to save some memory. BME280 into forced mode for better measures in WX station as per datasheet.
291223 Changed from ASyncElegantOTA lib to ElegantOTA. Needs declaration in platformio.ini
281223 Added RS485 checks if sensors are alive. Report state via new MQTT topics. 
261223 Added APRS digipeating possibility
251223 Rearranging some routines. Added sealevel pressure correction. New WITH_SEALEVELPRESSURE parameter inn settings.
241223 Correcting some small bugs. Adding NTP, WiFi RSSI
161223 Rearranging functions. Adding SHT31 support. Modifying MQTTpublish() to add a systematical connection to broker.
101223 Modified Web server so it displays only the available datas. Preparing SHT31 sensor support
200823 Added OTA and Web page server.
160823 Switched from Arduino IDE to platformIO. added wind_rs485 library
070723 Added static/DHCP address choice.
060723 Added display of modes set at boot. Removed IF_WIFI.
030723 Added APRS-IS protocol. Modified other functions to be compatible.
300623 Added MQTT protocol. Small bugs corrected. 
260623 Removed light sensor. Rebuild of build_LoRaString().
250623 Removed interrupt handling for the anemometer and vane routine. Replaced with RS485 ModBus protocol wind sensors.
210623 Back to interrupts in lieu of digitalread(). Using native BME280 library
200623 Added Wunderground support
190623 Test with digitalread to get ride of the interrupt bug
***************************************************/

#include <Arduino.h>
#include <logger.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <NTP.h>                    // github.com/sstaub/NTP.git
#include <TickTwo.h>
#include "index.htm"
#include "utils.h"
#include "BOSCH_utils.h"
#include "SHT31_utils.h"
#include "wind_rs485.h"
#include "display.h"
#include "settings.h"
#include "pins_config.h"
#include "APRS_utils.h"
#include "WiFi_utils.h"
#include "MQTT_utils.h"
#include "WG_utils.h"
#include "power_utils.h"
#if defined LILYGO_T3_V1_6
  #include "battery_utils.h"
#endif

/********************************************************************
 _____              __ _                       _   _             
/  __ \            / _(_)                     | | (_)            
| /  \/ ___  _ __ | |_ _  __ _ _   _ _ __ __ _| |_ _  ___  _ __  
| |    / _ \| '_ \|  _| |/ _` | | | | '__/ _` | __| |/ _ \| '_ \ 
| \__/\ (_) | | | | | | | (_| | |_| | | | (_| | |_| | (_) | | | |
 \____/\___/|_| |_|_| |_|\__, |\__,_|_|  \__,_|\__|_|\___/|_| |_|
                          __/ |                                  
                         |___/                                   
********************************************************************/

String SOFTWARE_DATE = "2024.03.15";

// ############ define counters ################
byte seconds;                       // When it hits 60, increase the current minute
byte seconds_2m;                    // Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes;                       // Keeps track of where we are in various arrays of data
byte minutes_10m;                   // Keeps track of where we are in wind gust/dir over last 10 minutes array of data
byte hours = 0;
int TXcounter;                     // counter for the period betweeb LoRa transmissions
String upTime;
bool FIRSTLOOP = true;             // to check if it is the first loop
uint64_t microSecondsSinceBoot;
String Reset_Reason;                // String to hold the last RESET reason

// ############### LoRa ##################
String LoRaString   = "";
String APRSString   = "";
String APRSISString = "";

// ############ temperature, humidity, pressure & volatge variables ############
float tempC, tempF, humi, press, batteryVoltage;

// ############ rain sensor ##########
//#ifdef WITH_RAIN
    volatile float rain1h[60];                                            //60 floating numbers to keep track of 60 minutes of rain
    float rain1hmm = 0;                                                   // rain during last 1 hour
    volatile float rain24hmm = 0;                                         //
    volatile unsigned int rainCount = 0, oldrainCount = 0;  // used in rain counter
//#endif  // end if WITH_RAIN

// ############# wind sensors ########## 
//#ifdef WITH_WIND
  HardwareSerial rs485Serial(1);            // declare UART0
  bool RS485WindSpeedSensorTimeout = false; // flag to check if the wind sensor is answering
  bool RS485WindDirSensorTimeout   = false; // flag to check if the wind sensor is answering
  int currentDir                   = 0;     // instant wind direction ModBus value in 8 direction values 
  int winDirAvg_2min                = 0;     // 2 minute average wind direction in degrees 0-360
  float currentSpeedms             = 0;     // instant wind speed ModBus value in m/s
  float currentSpeedKmh            = 0;     // instant wind speed ModBus value in km/h
  float windSpeed_avg2m            = 0;     // 2 minute average wind speed in km/h calculate from above array
  float windgustSpeed              = 0;     // wind gust speed max value
  int windgustDir                  = 0;     // wind gust dir max value
  float windgust_10m[10];                   // array of 10 floats to keep track of 10 minute max gust in m/s
  int windgustDir_10m[10];                  // array of 10 float values to keep track of the wind gust direction
//#endif

String LATITUDE, LONGITUDE;

#ifdef WITH_WIFI
  // Create AsyncWebServer object on port 80
  AsyncWebServer server(80);
  // NTP
  WiFiUDP wifiUdp;
  NTP ntp(wifiUdp);
#endif

int       screenBrightness    = 255;
logging::Logger               logger;

void mainloop();
TickTwo timer1(mainloop, 1000, 0, MILLIS);

/*****************************
 _____      _               
/  ___|    | |              
\ `--.  ___| |_ _   _ _ __  
 `--. \/ _ \ __| | | | '_ \ 
/\__/ /  __/ |_| |_| | |_) |
\____/ \___|\__|\__,_| .__/ 
                     | |    
                     |_|    
******************************/
void setup() {
    Serial.begin(115200);
    setup_display();
    #ifdef HAS_PMU
      POWER_Utils::setup();
      POWER_Utils::lowerCpuFrequency();
    #endif
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "Main", "WX station by TK5EP v %s", SOFTWARE_DATE);
    show_display(" APRS LoRa", "", "      WX station", "", " TK5EP Patrick EGLOFF","  version " + SOFTWARE_DATE, 4000);

    // define pin to change RS485 address.
    pinMode(PIN_CHANGE_ADDRESS,INPUT_PULLDOWN);

    #ifdef WITH_WIND
      wind_rs485::init();     // init the RS485 bus
      // Change address of wind sensor at startup
      // connect only one sensor on bus and close the switch wired on IO25. Green LED must light and press reset
      if (digitalRead(PIN_CHANGE_ADDRESS) == HIGH) {
          //Serial.println("ModBus address change");
          logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RS485", "Sensor address change procedure.");
          wind_rs485::changeRS485address();
      }
    #endif
    #ifdef DEBUG_I2C
      Utils::scanI2Cdevice();
    #endif
    // check & convert latitude and longitude in APRS format
    if (latitude == 0 && longitude == 0) {
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "Main", "Geographical coordinates not set, please correct this!");
      show_display("ERROR", "", "Coordinates not set!", "Correct this", "Halting..");
      while(1);
    }
    else {
      LATITUDE   = Utils::lat2APRS(latitude);
      LONGITUDE  = Utils::lng2APRS(longitude);
    }
    // check if callsign have been set
    if ( CALLSIGN == "NOCALL-13" ) {
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "Main", "Callsign set to NOCALL, please correct this");
      show_display("ERROR", "", "NOCALL-13 set!", "Correct this", "Halting..");
      while(1);
    }

    Reset_Reason = Utils::get_reset_reason(esp_reset_reason());
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RESET", "Last reset reason was %s", Reset_Reason);

    // init rain sensor interrupt pin
    #ifdef WITH_RAIN
        pinMode(interruptPinRain, INPUT_PULLUP);  // rain sensor
        attachInterrupt(digitalPinToInterrupt(interruptPinRain), Utils::IncRain, FALLING);
    #endif
    #if defined(WITH_BME280) || defined(WITH_BME680) || defined(WITH_BMP280)
        BOSCH_Utils::init();
    #endif
    #ifdef WITH_SHT31
        SHT31_Utils::init();
    #endif


    #ifdef WITH_APRS_LORA    
        // Start LoRa module
        APRS_Utils::LORAsetup();
    #endif
    
    #ifdef WITH_WIFI
        WiFi_Utils::connect();    // connect to WiFi AP
        // NTP
        ntp.ruleDST(DSTzone, DSTweek, DSTwday, DSTmonth, DSTwday, DSToffset); 
        ntp.ruleSTD(STDzone, STDweek, STDwday, STDmonth, STDwday, STDoffset); 
        ntp.begin();
        // OTA
        ElegantOTA.begin(&server, OTA_username, OTA_password);
        // WEBSERVER
        server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
          request->send_P(200, "text/html", index_html, WiFi_Utils::processor);
        });

        // display free heap memory
        server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "Free heap : " + String(ESP.getFreeHeap()));
        });

        server.begin();
    #endif

    timer1.start();
} // END SETUP

/*******************************
 _____                 _   _                 
|  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
| |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
|  _|| |_| | | | | (__| |_| | (_) | | | \__ \
|_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/ 

*******************************/
void mainloop() {
    #ifdef DEBUG_RAIN
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "RAIN", "Raincount : %s",String(rainCount));
    #endif

    // read wind sensors anc calculate wind infos
    #ifdef WITH_WIND
        RS485WindSpeedSensorTimeout = false;    // set the timeout flags
        RS485WindDirSensorTimeout   = false;
        // read wind speed and direction
        currentSpeedms  = wind_rs485::readWindSpeed(AddressSpeedSensor);    // read the wind speed, returns m/s or -1 if no answer from sensor
        delay(200);                                                         // needed to chain sensor readings
        currentDir      = wind_rs485::readWindDirection(AddressDirSensor);  // read the wind direction, returns degrees 0-360 or -1 if no answer from sensor
        // check if the ModBus sensors are answering
        if (currentSpeedms == -1)                                          // if we receive a timeout warning
        {
          logger.log(logging::LoggerLevel::LOGGER_LEVEL_WARN, "RS485", "Wind Speed ModBus timeout !");
          currentSpeedKmh = 0;
          RS485WindSpeedSensorTimeout = true;                               // set the timeout flag
        }
        else currentSpeedKmh = currentSpeedms * 3.6;                       // convert into km/h
          
        if (currentDir == -1 )                                              // if we receive a timeout warning
        {
          logger.log(logging::LoggerLevel::LOGGER_LEVEL_WARN, "RS485", "Wind direction ModBus timeout !");
          currentDir = 0 ;
          RS485WindDirSensorTimeout = true;
        }
        // compute all wind datas
        Utils::CalcWind();
    #endif

    // reactivate display 5s before TXing
    if ((TXcounter == TXPERIOD - 5) && (ECOMODE == 2 )) display_toggle(true);  
    // we send out datas every TXPERIOD and at boot
    if (++TXcounter == TXPERIOD || FIRSTLOOP) {       // if TX period has been reached and at boot
      TXcounter = 0;                                  // TXcounter reset
      // Battery measurement
      #if defined LILYGO_T3_V1_6
          batteryVoltage = BATTERY_Utils::checkBattery();
          #ifdef DEBUG_BAT
            logger.log( logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "BATTERY","Voltage is : %sV", String(batteryVoltage) );
          #endif
      #endif
      
      #ifdef HAS_PMU
          batteryVoltage = POWER_Utils::getBatteryVoltage();  // get battery voltage. 0.00V if not connected
          POWER_Utils::handleChargingLed();                   // handle charging LED
          //POWER_Utils::enableChgLed();                       // charging LED test
          #ifdef DEBUG_BAT
          if (POWER_Utils::isBatteryConnected()) {
              logger.log( logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "BATTERY","Battery is connected" );
              logger.log( logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "BATTERY","Voltage is : %sV", String(batteryVoltage) );
              if (POWER_Utils::isCharging()){
                logger.log( logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "BATTERY","Battery is charging" );
              }
              else {
                logger.log( logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "BATTERY","Battery not charging" );
              }
          }
          else {
            logger.log( logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "BATTERY","Battery not connected" );
          }
          #endif
      #endif

      // read BOSCH sensor if exists
      #if defined(WITH_BME280) || defined(WITH_BME680) || defined(WITH_BMP280) 
        BOSCH_Utils::read();
      #endif
      // read SHT31 if exists
      #ifdef WITH_SHT31
        SHT31_Utils::read();   
      #endif

      // prepare the APRS packet
      APRSString = APRS_Utils::build_APRSbeacon();
      // if APRS LoRa is activated send it
      #ifdef WITH_APRS_LORA
        APRS_Utils::send2APRS_LoRa(APRSString);
        // if it is at boot, send also a STATUS frame to clean old infos
        if (FIRSTLOOP) {
          APRS_Utils::send2APRS_LoRa("WX station by TK5EP v. " + SOFTWARE_DATE,true);
          // clear the flag, we're not anymore in first loop
          FIRSTLOOP = false;                                      
        }
      #endif

      // if WiFi has been activated, push datas in WiFI dependent protocols
      #ifdef WITH_WIFI
        // if not connected, reconnect to WiFi
        if (WiFi.status() != 3) WiFi_Utils::connect();
        // if connected to WiFi
        if (WiFi.status() == 3) {
          // update NTP time
          ntp.update();
          #ifdef DEBUG_NTP
            logger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "NTP","Time is : %s", String(ntp.formattedTime("%A %d %B %Y %T")).c_str());
          #endif
          // if APRS-IS is wanted, send out packet
          #ifdef WITH_APRS_IS
            APRS_Utils::send2APRS_IS(APRSString);
            // if FIRSTLOOP senad also STATUS packet to cleanup old one
            if (FIRSTLOOP) {
              APRS_Utils::send2APRS_IS("WX station by TK5EP v. "+ SOFTWARE_DATE,true);
              FIRSTLOOP = false;
            }
          #endif
          // if WUNDERGROUND is wanted, push datas
          #ifdef WITH_WUNDERGROUND
            WG_Utils::send();
          #endif
          // if MQTT protocol declared, transmit to MQTT broker
          #ifdef WITH_MQTT
            MQTT_Utils::publish();
          #endif
        }
      #endif

      // if ECOMODE is set to permanent than display OFF to save battery
      if (ECOMODE != 0 && !FIRSTLOOP)
        {
          display_toggle(false);
        }

    }  // END if TXperiod or first loop
    
    // update OLED display every second
    Utils::PrintOLED();
    
    // every minute do what has to be done
    if (++seconds > 59) {                     // when a minute is past
            seconds = 0;
            #ifdef WITH_WIND
                  // reset the stored values in array for next min
                  windgust_10m[minutes_10m+1] = 0;
                  windgustDir_10m[minutes_10m+1] = 0;
            #endif
            #ifdef WITH_RAIN
                  Utils::CalcRain();              // do all rain calculations
                  rainCount=0;                    // resets rain bucket counter every min
                  rain1h[minutes] = 0;            // resets last minute array value
            #endif

            // counts every 10 minutes
            if (++minutes_10m > 9) {
              minutes_10m = 0;
            }

            // counts minutes and resets at hour
            if (++minutes > 59) {
                minutes = 0;
                #ifdef WITH_RAIN
                  // reset the last 60 min rain accumulation to make a new calc
                  rain1hmm = 0;
                #endif
              // counts the hours for a full day
              if (++hours > 23) {
                  hours = 0;
                  #ifdef WITH_RAIN
                      // reset daily rainfall
                      rain24hmm = 0;
                  #endif
                }  
            }
         
    } // end minute count

    // every 10 seconds 
    if (seconds % 10 == 0) {
      uint64_t microSecondsSinceBoot = esp_timer_get_time();  
      upTime = Utils::delayToString(microSecondsSinceBoot / 1000);
    } // END every 10s

    //counts every 2 min
    if (++seconds_2m > 119) {
      seconds_2m = 0;
    }
    
    #ifdef DEBUG_TIME
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "TIME", "millis,sec/2min/min/10min/TX");
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "TIME", "%d,%d/%d/%d/%d/%d",millis(),seconds,seconds_2m,minutes,minutes_10m,TXcounter);
    #endif
  }

/*****************************
 _                       
| |                      
| |     ___   ___  _ __  
| |    / _ \ / _ \| '_ \ 
| |___| (_) | (_) | |_) |
\_____/\___/ \___/| .__/ 
                  | |    
                  |_|    
*****************************/
void loop() {
  // check OTA state
  ElegantOTA.loop();
  // call main loop with ticker (1s rate)
  timer1.update();
}  // end loop() 