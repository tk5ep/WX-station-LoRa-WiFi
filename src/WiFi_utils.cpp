
#include "settings.h"
#include "display.h"
#include <logger.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>           // by Nick O'Leary
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <NTP.h>                    // github.com/sstaub/NTP.git
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
extern AsyncEventSource events;



namespace WiFi_Utils {
   /******************************************
  * processor for the Web page 
  *******************************************/
  String processor(const String& var){
  if (var == "RESET")         return String(Reset_Reason);
  if (var == "CALLSIGN")      return String(CALLSIGN);
  if (var == "FREQUENCY")     return String(TXFREQUENCY,3);
  if (var == "POWER")         return String(TXPOWER);  
  if (var == "VERSION_DATE")  return String(SOFTWARE_DATE);
  if (var == "TEMPERATURE")   return String(tempC);
  if (var == "HUMIDITY")      return String(humi);
  if (var == "PRESSURE")      return String(press);
  if (var == "WINDSPEED")     return String(windSpeed_avg2m);
  if (var == "WINDDIR")       return String(winDirAvg_2min);
  if (var == "GUSTSPEED")     return String(windgustSpeed);
  if (var == "GUSTDIR")       return String(windgustDir);
  if (var == "RAIN1H")        return String(rain1hmm);
  if (var == "RAIN24H")       return String(rain24hmm);
  if (var == "TXPERIOD")      return String(TXPERIOD);
  if (var == "INTBATVOLT")    return String(batteryVoltage);
  if (var == "UPTIME")        return String(upTime);
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
      if (var == "SSID") return String(WiFi.SSID());
      if (var == "RSSI") return String(WiFi.RSSI());
      if (var == "NTPDATE") return String(ntp.formattedTime("%d %B %Y"));
      if (var == "NTPTIME") return String(ntp.formattedTime("%T"));
      //if (var == "RESTART") return String(ntp.formattedTime("%T"));
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
            ntp.ruleDST(DSTzone, DSTweek, DSTwday, DSTmonth, DSThour, DSToffset); 
            ntp.ruleSTD(STDzone, STDweek, STDwday, STDmonth, STDhour, STDoffset); 
            //ntp.ruleSTD("CET", Last, Sun, Oct, 3, 60); 
            //ntp.ruleSTD("CET", 0, 0, 10, 3, 60); 
            ntp.begin();
            #ifdef DEBUG_NTP
                logger.log( logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "NTP", "STD time : %s", ntp.ruleSTD() );
                logger.log( logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "NTP", "DST time : %s", ntp.ruleDST() );
                logger.log( logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "NTP", "is DST : %s", String (ntp.isDST()) );
            #endif
            // OTA
            ElegantOTA.begin(&server, OTA_username, OTA_password);
            // WEBSERVER
            server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(200, "text/html", index_html, processor);
            });
            // display free heap memory
            server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(200, "text/plain", "Free heap : " + String(ESP.getFreeHeap()));
            });

            // Handle Web Server Events
            events.onConnect([](AsyncEventSourceClient *client){
              if(client->lastId()){
                logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "WEB", "Client reconnected! Last message ID that it got is: %u",client->lastId());
              }

              client->send("hi!", NULL, millis(), 10000);
            });

            server.addHandler(&events);
            server.begin();
  }

} // end spacename