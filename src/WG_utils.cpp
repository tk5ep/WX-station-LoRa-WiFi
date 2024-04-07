#include "WG_utils.h"
#include "settings.h"
#include "display.h"
#include <logger.h>
#include <WiFi.h>

extern logging::Logger  logger;
extern float humi;
extern float press;
extern float tempC;
extern byte rain1hAPRS;
extern int winDirAvg_2min;
extern float windSpeed_avg2m;
extern int windgustDir;
extern float windgustSpeed;
extern float rain1hmm;
extern volatile float rain24hmm;
extern WiFiClient client;

/**************************************************************************
 upload to Wunderground
***************************************************************************
Documentation :
https://support.weather.com/s/article/PWS-Upload-Protocol?language=en_US
Upload :
https://weatherstation.wunderground.com/weatherstation/updateweatherstation.php?ID=XXXXX&PASSWORD=YYYYYYY&dateutc=now&humidity=59&action=updateraw

action [action=updateraw] -- always supply this parameter to indicate you are making a weather observation upload
ID [ID as registered by wunderground.com]
PASSWORD [Station Key registered with this PWS ID, case sensitive]
dateutc - [YYYY-MM-DD HH:MM:SS (mysql format)] In Universal Coordinated Time (UTC) Not local time
winddir - [0-360 instantaneous wind direction]
windspeedmph - [mph instantaneous wind speed]
windgustmph - [mph current wind gust, using software specific time period]
windgustdir - [0-360 using software specific time period]
windspdmph_avg2m  - [mph 2 minute average wind speed mph]
winDirAvg_2min - [0-360 2 minute average wind direction]
windgustmph_10m - [mph past 10 minutes wind gust mph ]
windgustdir_10m - [0-360 past 10 minutes wind gust direction]
humidity - [% outdoor humidity 0-100%]
dewptf- [F outdoor dewpoint F]
tempf - [F outdoor temperature]
* for extra outdoor sensors use temp2f, temp3f, and so on
rainin - [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
dailyrainin - [rain inches so far today in local time]
baromin - [barometric pressure inches]
*/

namespace WG_Utils {
    void send() {
        // connect to wunderground
        if (!client.connect(WG_server, 80)) {
            //Serial.println(F("Send2Wunder Failed"));
            logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "WG", "Send2Wunder Failed");
            return;
        } else {
            //Serial.println(F("Connected. WeatherUnderground page updating...."));
            logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "WG", "Connected. WeatherUnderground page updating");
        }

        float tempF = (tempC * 1.8) + 32;  // conversion Celsius to Fahrenheit
        float dewptf = (DewPoint(tempC, humi) * 1.8) + 32;  // compute dewpoint and convert Celsius to Fahrenheit

        String url = "/weatherstation/updateweatherstation.php?ID=";
        url += WG_ID;
        url += "&PASSWORD=";
        url += WG_PWD;
        url += "&dateutc=now&tempf=";
        url += tempF;
        url += "&humidity=";
        url += humi;
        url += "&dewptf=";
        url += dewptf;
        #if defined(WITH_BME280) || defined(WITH_BME680)
            url += "&baromin=";
            url += (press * 0.02953f);  // 1 hPa = 0.02953 inHg
        #endif    
        #ifdef WITH_RAIN
            url += "&rainin=";
            url += String(rain1hmm/25.3998,3);  // mm to inches
            url += "&dailyrainin=";
            url += String(rain24hmm/25.3998,3);
        #endif
        #ifdef WITH_WIND
            url += "&windspeedmph=";
            url += (windSpeed_avg2m * 0.62);
            url += "&winddir=";
            url += winDirAvg_2min;
            url += "&windgustmph=";
            url += (windgustSpeed * 0.62);
            url += "&windgustdir=";
            url += windgustDir;
        #endif
        url += "&action=updateraw";

        logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "WG", "Sending datas to WG server");
        client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: weatherstation.wunderground.com\r\n" + "User-Agent: G6EJDFailureDetectionFunction\r\n" + "Connection: close\r\n\r\n");

        #ifdef DEBUG_WG
            delay(2000);  // let the time to server to answer
            logger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "WG", "URL %s",url.c_str());
            logger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "WG", "Server answers :");
            while (client.available()) {
                String line = client.readStringUntil('\r');
                Serial.print(line);
            }
        #endif
        client.stop();
    }

/*****************************************************
 function : dewpoint calculations https://gist.github.com/Mausy5043/4179a715d616e6ad8a4eababee7e0281
 input : double celcius, double humidity
 returns : double dewpoint in °C
*****************************************************/
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
}