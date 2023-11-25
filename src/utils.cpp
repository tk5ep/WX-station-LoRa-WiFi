/********************************************************
 * functions for the LoRa_WX by TK5EP
********************************************************/

#include <Arduino.h>
#include "utils.h"

// convert time in ms to day hour min sec
String delayToString(unsigned long time_ms) {
	char buf[64];
	String s;

    unsigned long seconds = time_ms / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    unsigned long days = hours / 24;
  
    seconds %= 60;
    minutes %= 60;
    hours %= 24;

    sprintf_P(buf, PSTR("%03dd:%02dh:%02dm:%02ds"),days,hours,minutes,seconds );
    s = buf;
	return s;
}

// function : dewpoint calculations https://gist.github.com/Mausy5043/4179a715d616e6ad8a4eababee7e0281
// input : double celcius, double humidity
// returns : double dewpoint in °C
double DewPoint(double celsius, double humidity) {
  double RATIO = 373.15 / (273.15 + celsius); 
  double SUM = -7.90298 * (RATIO - 1);
  SUM += 5.02808 * log10(RATIO);
  SUM += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / RATIO))) - 1);
  SUM += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1);
  SUM += log10(1013.246);
  double VP = pow(10, SUM - 3) * humidity;
  double T = log(VP / 0.61078);  // temp var
  return (241.88 * T) / (17.558 - T);
}


// function : converts degrees 0-360 into compass direction by 22.5 sectors ex : N = 348.75 to 11.25, NNE = 11.25 to 33.75
// input : int degrees 0-360
// returns : String cardinal direction "N"
String Deg2Compass(int deg) {
    String dir[17]={"N","NNE","NE","ENE","E","ESE","SE","SSE","S","SSW","SW","WSW","W","WNW","NW","NNW","N"};
    int d = int((deg + 11.25) /22.5);
    return dir[d];
}


// function : get the ESP32 reset reason
// input : uint8_t answer of esp_reset_reason() 
// returns : String readable reset reason
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