#include "utils.h"
#include <Arduino.h>
#include <logger.h>
#include <Wire.h>
#include "display.h"
#include <math.h>
#include "settings.h"

extern logging::Logger  logger;

extern float press;
extern float tempC;
extern float humi;
extern float windSpeed_avg2m;
extern int windgustDir;
extern float windgustSpeed;
extern int TXcounter;
extern byte seconds_2m;
extern byte minutes_10m;
extern byte minutes;
extern bool RS485WindSpeedSensorTimeout; // flag to check if the wind sensor is answering
extern bool RS485WindDirSensorTimeout; // flag to check if the wind sensor is answering
extern int currentDir;     // instant wind direction ModBus value in 8 direction values 
extern int winDirAvg_2min;     // 2 minute average wind direction in degrees 0-360
volatile int windDirAvg[120];             // array of 120 ints to keep track of 2 minute average
volatile double windDirAvgX[120];
volatile double windDirAvgY[120];
volatile float windSpeedAvg[120];         // to keep track of last 120 wind speed measures
extern float currentSpeedms;     // instant wind speed ModBus value in m/s
extern float currentSpeedKmh;     // instant wind speed ModBus value in km/h
extern float windSpeed_avg2m;     // 2 minute average wind speed in km/h calculate from above array
extern float windgustSpeed;     // wind gust speed max value
extern int windgustDir;     // wind gust dir max value
extern float windgust_10m[10];                   // array of 10 floats to keep track of 10 minute max gust in m/s
extern int windgustDir_10m[10];                  // array of 10 float values to keep track of the wind gust direction
extern float rain1hmm;
extern volatile float rain24hmm;
extern volatile unsigned int rainCount;
extern volatile unsigned int oldrainCount;
extern volatile float rain1h[60];
extern byte rain1hAPRS;

volatile unsigned long rainlast;

namespace Utils {
/***********************************************
 convert time in ms to day hour min sec
***********************************************/
String delayToString(unsigned long time_ms) {
	char buf[64];
	String s;
    int seconds = time_ms / 1000;
    int minutes = seconds / 60;
    int hours = minutes / 60;
    int days = hours / 24;
 
    seconds %= 60;
    minutes %= 60;
    hours %= 24;

    sprintf_P(buf, PSTR("%03dd:%02dh:%02dm:%02ds"),days,hours,minutes,seconds );
    s = buf;
	return s;
}

void PrintOLED() {
    String line1,line2,line3,line4,line5,line6;
    // formatting line1
    if (RS485WindSpeedSensorTimeout || RS485WindDirSensorTimeout) {
      line1 = CALLSIGN;
    }
    else line1 = CALLSIGN;
    
    // formatting line2
    #if defined(WITH_BME280) || defined(WITH_BME680) || defined(WITH_BMP280) // if we have a pressure to display
      if(press>=1000) line2 = "PRES: "+String(press,1)+ " hPa  " + (TXPERIOD-TXcounter);
      else line2 = "PRES: "+String(press,1)+ " hPa   " + (TXPERIOD-TXcounter);
    #else
      line2 = "TX in " + String((TXPERIOD-TXcounter)) + "s";
    #endif
    // formatting line3
    line3 = "TEMP: " + String(tempC,1) + " C";
    // formatting line4
    #if defined(WITH_BME280) || defined(WITH_BME680) || defined(WITH_SHT31)
        line4 = "HUMI: " + String(humi,1) + " %";
    #else
        line4="";
    #endif
    #ifdef WITH_RAIN
    // formatting line    
    line5 = "RAIN: " + String(rain1hmm)+ " mm/h";
    #else
      line5="";
    #endif
    // formatting line6    
    #ifdef WITH_WIND
      line6 = "WIND: " + String(windSpeed_avg2m,1) + " km/h " + String(winDirAvg_2min) + (char)247;
    #else
      line6="";
    #endif
    show_display (line1,line2,line3,line4,line5,line6);
}

/*******************************************************
make all wind calculations
********************************************************/
  void CalcWind() {
    double windDirAvgX_2min = 0.00;
    double windDirAvgY_2min = 0.00;
    double windDirX = 0.00;
    double windDirY = 0.00;
    double theta = 0.00;

    // adds current 1s reading in float km/h into 2 min array
    windSpeedAvg[seconds_2m] = currentSpeedKmh;  
    // calculates the average wind speed from last 120 measures (2 min)
    for (int i = 0; i < 120; i++) {
      windSpeed_avg2m += windSpeedAvg[i];
    }
    windSpeed_avg2m /= 120.0;        // 2 min average wind speed

     // calculate wind direction
    currentDir = currentDir * 45;    // transforms 0-7 values from sensor to 0-360 degrees
    windDirAvg[seconds_2m] = currentDir;

    // calculate circular mean direction
    // convert deg to radian, needed for sin() & cos()
    theta =  currentDir /180.0 * PI;
    // get X, Y values of vector
    windDirX = cos(theta);
    windDirY = sin(theta);
    // push these values into the 2min array
    windDirAvgX[seconds_2m] = windDirX;
    windDirAvgY[seconds_2m] = windDirY;

    // calculates the average wind dir from last 120 measures (2 min)
    for (int i = 0; i < 120; i++) {
      windDirAvgX_2min += windDirAvgX[i];
      windDirAvgY_2min += windDirAvgY[i];
    }
    windDirAvgX_2min /= 120.0;
    windDirAvgY_2min /= 120.0;
 
    // get the angle in degrees from mean X,Y
    winDirAvg_2min = atan2(windDirAvgY_2min, windDirAvgX_2min) / PI*180;
    // result is -180 to 180. change this to 0-360.  
    if(winDirAvg_2min < 0) winDirAvg_2min += 360;
    /*
    // debug 
    Serial.print("Current dir : ");
    Serial.println(currentDir);
    Serial.print("Instant X   : ");
    Serial.println(windDirAvgX[seconds_2m],5);
    Serial.print("Instant Y   : ");
    Serial.println(windDirAvgY[seconds_2m],5);
    Serial.print("Mean X      : ");
    Serial.println(windDirAvgX_2min,5);
    Serial.print("Mean Y      : ");
    Serial.println(windDirAvgY_2min,5);
    Serial.print("Avg dir     : ");
    Serial.println(winDirAvg_2min);
    */
   
    // do we have a wind gust for the minute ?
    if (currentSpeedKmh > windgust_10m[minutes_10m]) {
      windgust_10m[minutes_10m] = currentSpeedKmh;            // if yes store new values for speed into 10 floats array
      windgustDir_10m[minutes_10m] = currentDir;              // same for gust direction
    }
    // calculate wind gust speed & direction
    windgustSpeed = 0;
    windgustDir = 0;
    for (int i = 0; i < 10; i++) {
        //windgustSpeed = max(windgust_10m[i],windgustSpeed);  // get the max values from 10 last
        if (windgust_10m[i] >= windgustSpeed) {
            windgustSpeed = windgust_10m[i];                  // max speed
            windgustDir = windgustDir_10m[i];                 // corresponding dir
        }
    }

    #ifdef DEBUG_WIND
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "CalcWind", "currentSpeed km/h : %s km/h",String(currentSpeedKmh));
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "CalcWind", "windSpeed_avg2m : %s km/h",String(currentSpeedKmh));
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "CalcWind", "Gust speed : %s km/h",String(windgustSpeed));
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "CalcWind", "Gust dir : %s deg",String(windgustDir));
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "CalcWind", "CurrentDir : %s deg",String(currentDir));
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "CalcWind", "winDirAvg_2min : %s deg",String(winDirAvg_2min));
      //Serial.println();
    #endif
  }

/***************************************
// counts numbers of rain buckets, uses soft debouncing
****************************************/
#ifdef WITH_RAIN
    void IRAM_ATTR IncRain() {
    //volatile unsigned long rainlast;
    if (millis() - rainlast > RAINDEBOUNCE)  // suppress incoming bounces coming in before set debounce time
      {
        rainCount++;          // increment rain counter
        rainlast = millis();  // resets debounce time
      }
    }
#endif

/***************************************
    // calculates daily rain fall
****************************************/
#ifdef WITH_RAIN
    void CalcRain() { 
        rain24hmm       += rainCount * rainBucketCont;  // daily rainfall
        rain1h[minutes] += rainCount * rainBucketCont;  // rain quantity during last minute
        
        for (int i = 0; i < 60; i++) {   // calc total rainfall for last sliding 60 min (not last hour)
          rain1hmm += rain1h[i];
        }
        
        #ifdef DEBUG_RAIN
          logger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "RAIN", "RainCount : %s during min %s for %s mm/min",String(rainCount), String(minutes), String(rain1h[minutes]) );
          logger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "RAIN", "Rain/h : %s mm/h %s mm/24h", String(rain1hmm), String(rain24hmm));
        #endif
        
      }
#endif

/***************************************************
 function : get the ESP32 reset reason
 input : uint8_t answer of esp_reset_reason() 
 returns : String readable reset reason
***************************************************/
String get_reset_reason(uint8_t reason)
{
  String ret ="";
  switch ( reason)
  {
    // https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/reset_reason.html
    case 1 : ret = "Vbat power ON";break;          /**<1, Vbat power on reset*/
    case 3 : ret = "Software";break;               /**<3, Software reset digital core*/
    case 4 : ret = "Watch dog";break;             /**<4, Legacy watch dog reset digital core*/
    case 5 : ret = "Deep Sleep";break;              /**<5, Deep Sleep reset digital core*/
    case 6 : ret = "Reset by SLC module";break;             /**<6, Reset by SLC module, reset digital core*/
    case 7 : ret = "Timer Group0 Watch dog";break;       /**<7, Timer Group0 Watch dog reset digital core*/
    case 8 : ret = "Timer Group1 Watch dog";break;       /**<8, Timer Group1 Watch dog reset digital core*/
    case 9 : ret = "RTC Watch dog";break;          /**<9, RTC Watch dog Reset digital core*/
    case 10 : ret = "Intrusion";break;       /**<10, Instrusion tested to reset CPU*/
    case 11 : ret = "Time Group";break;       /**<11, Time Group reset CPU*/
    case 12 : ret = "Software reset CPU";break;          /**<12, Software reset CPU*/
    case 13 : ret = "RTC Watch dog";break;      /**<13, RTC Watch dog Reset CPU*/
    case 14 : ret = "EXT_CPU_RESET";break;         /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : ret = "Unstable VDD voltage";break;/**<15, Reset when the vdd voltage is not stable*/
    case 16 : ret = "RTCWDT_RTC_RESET";break;      /**<16, RTC Watch dog reset digital core and rtc module*/
    default : ret = "NO_MEAN";
  }
  return ret;
}

/************************
 convert latitude from DD.MMMM to DDMM.MM for APRS format
*************************/
String lat2APRS (float ddmmm) {
  char dir[2];
  char buffer[20];
  // get direction N/S
  if (ddmmm >=0) strcpy(dir,"N");
  else strcpy(dir,"S");
  // get deg 
  ddmmm = abs(ddmmm);
  int deg = ddmmm;
  // get min
  float min = (ddmmm-deg) * 60;
  // format to APRS format
  sprintf(buffer,"%02d%02d.%02d%s",deg, int(min), int(min*100.0)%100, dir);
  //Serial.println(buffer);
  return String(buffer);
}

/************************
 convert longitude from DD.MMMM to DDMM.MM for APRS format
*************************/
String lng2APRS (float ddmmm) {
  char dir[2];
  char buffer[20];
  // get direction W/E
  if (ddmmm >=0) strcpy(dir,"E");
  else strcpy(dir,"W");
  // get deg 
  ddmmm = abs(ddmmm);
  int deg = ddmmm;
  // get min
  float min = (ddmmm-deg) * 60;
  // format to APRS format
  sprintf(buffer,"%03d%02d.%02d%s",deg, int(min), int(min*100.0)%100, dir);
  //Serial.println(buffer);
  return String(buffer);
}

/************************
 scan i2C devices
*************************/
int scanI2Cdevice(void)
{
  byte err, addr;
  int nDevices = 0;
  for (addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("I2C device found at address 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.print(addr, HEX);
      Serial.println(" !");
      nDevices++;
      if (addr == 0x3c) {
        Serial.println("0x3c probably OLED display");
      }
      if (addr == 0x3d) {
        Serial.println("0x3d probably OLED display");
      }
      if (addr == 0x76) {  
        Serial.println("0x76 probably BMExxx");
      }
      if (addr == 0x77) {  
        Serial.println("0x77 probably BMExxx");
      }
      if (addr == 0x44) {  
        Serial.println("0x44 probably SHT31");
      }
      if (addr == 0x45) {  
        Serial.println("0x45 probably SHT31");
      }
    } else if (err == 4) {
      Serial.print("Unknow error at address 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.println(addr, HEX);
    }
  }
  if (nDevices == 0)
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "i2C", "No i2C device found.");
  else
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "i2C", "i2C scanning done");
  return nDevices;
}

} // end spacenames