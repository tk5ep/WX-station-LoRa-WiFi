
#include "settings.h"
#include "display.h"
#include <logger.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>           // by Nick O'Leary
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include "NTP.h"                    // github.com/sstaub/NTP.git
#include "index.htm"

extern logging::Logger  logger;
extern NTP ntp;
extern String SOFTWARE_DATE;
extern String Reset_Reason;
extern float press;
extern float tempC;
extern float humi;
extern float rain1hmm;
extern volatile float rain24hmm;
extern float windSpeed_avg2m;
extern int winDirAvg_2min;
extern int windgustDir;
extern float windgustSpeed;
extern String upTime;
extern float batteryVoltage;
extern AsyncWebServer server;

namespace WiFi_Utils {
   /******************************************
  * processor for the Web page 
  *******************************************/
  String processor(const String& var){
  if (var == "RESET")       return String(Reset_Reason);
  if (var == "CALLSIGN")    return String(CALLSIGN);
  if (var == "FREQUENCY")   return String(TXFREQUENCY,3);
  if (var == "POWER")       return String(TXPOWER);  
  //if (var == "VERSION")     return String(SOFTWARE_VERSION);
  if (var == "VERSION_DATE") return String(SOFTWARE_DATE);


  // arranging column 2 & 3 depending sensor used
  if (var == "COL1") return "<div class=\"card\"><p><i class=\"fas fa-thermometer-half\"  style=\"color:#059e8a;\"></i> TEMPERATURE</p><p><span class=\"reading\"><span id=\"temp_html\">" +  String(tempC) + "</span> &deg;C</span></p></div>";

  if (var == "COL2") { 
      #if defined(WITH_BME280) || defined(WITH_BME680) || defined(WITH_SHT31)
        return "<div class=\"card\"><p><i class=\"fas fa-tint\"style=\"color:#00add6;\"></i> HUMIDITY</p><p><span class=\"reading\"><span id=\"hum_html\">" + String(humi) +    "</span> &percnt;</span></p></div>";
      #endif
      # if defined(WITH_BMP280)
        return "<div class=\"card\"><p><i class=\"fas fa-angle-double-down\" style=\"color:#e1e437;\"></i> PRESSURE</p><p><span class=\"reading\"><span id=\"press_html\">" + String(press) + "</span> hPa</span></p></div>";
      #endif
  }

  if (var == "COL3") {
      #if defined(WITH_BME280) || defined(WITH_BME680)// if BME280 or BME680 only, display pressure value SHT31 has no press sensor
            return "<div class=\"card\"><p><i class=\"fas fa-angle-double-down\" style=\"color:#e1e437;\"></i> PRESSURE</p><p><span class=\"reading\"><span id=\"press_html\">" + String(press) + "</span> hPa</span></p></div>";
      #endif  
  }



  #ifdef WITH_WIND
      if (var == "WINDSPEED") return "<div class=\"card\"><p><i class=\"fas fa-flag\" style=\"color:#059e8a;\"></i> WIND SPEED</p><p><span class=\"reading\"><span id=\"windspeed_html\">" + String(windSpeed_avg2m) + "</span> km/h</span></p></div>";
      if (var == "WINDDIR")   return "<div class=\"card\"><p><i class=\"fas fa-location-arrow\" style=\"color:#00add6;\"></i> WIND DIR</p><p><span class=\"reading\"><span id=\"winddir_html\">" + String(winDirAvg_2min) + "</span> &deg;</span></p></div>";
      if (var == "GUSTSPEED") return "<div class=\"card\"><p><i class=\"fas fa-flag\" style=\"color:#FF0000;\"></i> GUST SPEED</p><p><span class=\"reading\"><span id=\"gustspeed_html\">" + String(windgustSpeed) + "</span> km/h</span></p></div>";
      if (var == "GUSTDIR")   return "<div class=\"card\"><p><i class=\"fas fa-location-arrow\" style=\"color:#FF0000;\"></i> GUST DIR</p><p><span class=\"reading\"><span id=\"gustdir_html\">" + String(windgustDir) + "</span> &deg;</span></p></div>";
  #endif
 
  #ifdef WITH_RAIN
      if (var == "RAIN1H")    return "<div class=\"card\"><p><i class=\"fas fa-cloud-rain\" style=\"color:#059e8a;\"></i> RAIN 1h </p><p><span class=\"reading\"><span id=\"rain1h_html\">"  + String(rain1hmm)  + "</span> mm</span></p></div>";
      if (var == "RAIN24H")   return "<div class=\"card\"><p><i class=\"fas fa-cloud-rain\" style=\"color:#00add6;\"></i> RAIN 24h</p><p><span class=\"reading\"><span id=\"rain24h_html\">" + String(rain24hmm) + "</span> mm</span></p></div>";
  #endif

  // display some parameters
      if (var == "TXPERIOD")    return String(TXPERIOD);
      if (var == "INTBATVOLT")  return String(batteryVoltage);
      if (var == "UPTIME")      return String(upTime);
  #if defined WITH_APRS_LORA
      if (var == "WITHAPRS") return "<i class=\"fas fa-check\" style=\"color:#059e8a;\"></i>";
    #else
      if (var == "WITHAPRS") return "<i class=\"fas fa-times\" style=\"color:#FF0000;\"";
  #endif
  #if defined WITH_MQTT
      if (var == "WITHMQTT") return "<i class=\"fas fa-check\" style=\"color:#059e8a;\"></i>";
    #else
    if (var == "WITHMQTT") return "<i class=\"fas fa-times\" style=\"color:#FF0000;\"";
  #endif
  #if defined WITH_APRS_IS
      if (var == "WITHAPRSIS") return "<i class=\"fas fa-check\" style=\"color:#059e8a;\"></i>";
    #else
      if (var == "WITHAPRSIS") return "<i class=\"fas fa-times\" style=\"color:#FF0000;\"";
  #endif
  #if defined WITH_WUNDERGROUND
      if (var == "WITHWG") return "<i class=\"fas fa-check\" style=\"color:#059e8a;\"></i>";
    #else
      if (var == "WITHWG") return "<i class=\"fas fa-times\" style=\"color:#FF0000;\"></i>";
  #endif
  #ifdef WITH_WIFI
      if (var == "WIFI") return "<tr><td>WiFi</td><td colspan=\"2\">SSID : " + String(WiFi.SSID()) +                  "</td><td colspan=\"2\">RSSI : " + String(WiFi.RSSI()) + "dBm</td></tr>";
      if (var == "NTP") return "<tr><td>NTP</td>  <td colspan=\"2\">" + String(ntp.formattedTime("%d %B %Y")) + "</td><td colspan=\"2\">Time : "    + String(ntp.formattedTime("%T")) + "</td></tr>";
  #endif
  return String();
}

/**********************************
connect to WiFi
***********************************/
void connect() {
      // init WiFi if needed
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "WiFi", "WiFi trying to connect");
      show_display("WiFi", "", "Trying to connect..");
      
      // is we use a static IP, not DHCP
      #ifdef WITH_STATIC_IP 
          if (WiFi.config(local_IP, gateway, subnet, primaryDNS))
            { logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "WiFi", "STA configured"); }
          else 
            { logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "WiFi", "STA config failed"); } 
      #endif

      // Connect to WPA/WPA2 network.
      WiFi.begin(wifi_ssid, wifi_password);
      // try 20 WiFi connections
      int numberOfTries = 20;                 
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        if (numberOfTries <= 0) {
          break; 
          }
        else { numberOfTries--; }
      }
      // if connection succeeded
      if (WiFi.status() == WL_CONNECTED) {
        char buffer[24];
        sprintf(buffer,"IP : %u.%u.%u.%u", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "WiFi", "Connected to %s with RSSI %s dBm", buffer,String(WiFi.RSSI()));
        show_display("WiFi","", "Wifi connected to :",wifi_ssid,buffer,"RSSI : " + String(WiFi.RSSI()) + "dBm",4000);
      } else {
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "WiFi", "Connection failed");
        WiFi.disconnect();

        if (WiFi.status() == WL_NO_SSID_AVAIL) {
          logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "WiFi", "SSID not found");
          show_display("WiFi","", "Failed","SSID not found",4000);
        }
      }
  }

/**********************************
init the servers
***********************************/
void init() {
            // NTP
            ntp.ruleDST(DSTzone, DSTweek, DSTwday, DSTmonth, DSTwday, DSToffset); 
            ntp.ruleSTD(STDzone, STDweek, STDwday, STDmonth, STDwday, STDoffset); 
            ntp.begin();
            // OTA
            ElegantOTA.begin(&server, OTA_username, OTA_password);
            // WEBSERVER
            server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send_P(200, "text/html", index_html, processor);
            });
            // display free heap memory
            server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(200, "text/plain", "Free heap : " + String(ESP.getFreeHeap()));
            });
            server.begin();
  }

} // end spacename