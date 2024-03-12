#include "APRS_utils.h"
#include "settings.h"
#include "display.h"
#include "pins_config.h"
#include <logger.h>
#include <RadioLib.h>
#include <WiFi.h>

extern logging::Logger  logger;
extern float humi;
extern float press;
extern float tempC;
extern byte rain1hAPRS;
extern byte rain1hAPRS;
extern int windDir_avg2m;
extern float windSpeed_avg2m;
extern int windgustDir;
extern float windgustSpeed;
extern WiFiClient client;
extern float rain1hmm;
extern volatile float rain24hmm;
extern float batteryVoltage;
extern String LATITUDE;
extern String LONGITUDE;

#ifdef HAS_SX1268
    SX1268 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN ,RADIO_BUSY_PIN);
#else
    SX1278 radio = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);
#endif

namespace APRS_Utils {
  /**************************************************************************
   setup LORA module
  ***************************************************************************/
  void LORAsetup() {
    #ifdef HAS_SX1268
      int state = radio.begin(TXFREQUENCY);
      if (state == RADIOLIB_ERR_NONE) {
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "SX1268", "Initializing success");
        // set DIO2 RF switch
        if (radio.setDio2AsRfSwitch(true) != RADIOLIB_ERR_NONE) {
          logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "SX1268", "failed to set DIO2 as RF switch");
        }
      } else {
          logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "SX1268", "Initializing error with code : %s", String(state));
          while (true);
      }
    #else
      int state = radio.begin(TXFREQUENCY);
      if (state == RADIOLIB_ERR_NONE) {
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "SX1278", "Initializing success");
      } else {
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "SX1278", "Initializing error with code : %s", String(state));
        while (true);
      }
      #endif
      /*
      //radio.setDio1Action(setFlag);
      // set carrier frequency
      if (radio.setFrequency(TXFREQUENCY) == RADIOLIB_ERR_INVALID_FREQUENCY) {
        Serial.println(F("Selected frequency is invalid for this module!"));
        while (true);
      }
      */
      // set bandwidth to 250 kHz
      if (radio.setBandwidth(125.0) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
        //Serial.println(F("Selected bandwidth is invalid for this module!"));
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "SX12XX", "Selected bandwidth is invalid for this module!");
        while (true);
      }

      // set spreading factor to 12
      if (radio.setSpreadingFactor(12) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
        //Serial.println(F("Selected spreading factor is invalid for this module!"));
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "SX12XX", "Selected spreading factor is invalid for this module!");
        while (true);
      }

      // set coding rate to 6
      if (radio.setCodingRate(5) == RADIOLIB_ERR_INVALID_CODING_RATE) {
        //Serial.println(F("Selected coding rate is invalid for this module!"));
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "SX12XX", "Selected coding rate is invalid for this module!");
        while (true);
      }

      if (radio.setOutputPower(TXPOWER) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        //Serial.println(F("Selected output power is invalid for this module!"));
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "SX12XX", "Selected output power is invalid for this module!");
        show_display("TX APRS", "", "Wrong power setting!");
        while (true);
      }
      if (radio.setCurrentLimit(140) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
        //Serial.println(F("Selected current limit is invalid for this module!"));
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "SX12XX", "Selected current limit is invalid for this module!");
        while (true);
      }
  }

  /**************************************************************************
      build the APRS beacon string 
  ***************************************************************************/
  String build_APRSbeacon() {
        /* build string for LoRa APRS as per APRS protocol reference 
        ex !4112.28N/08403.57W_045/086g097t026r002p007P006h02b03241wDvs
        c = wind direction in degrees
        s = wind speed over 1 min in mph (we send over 2min like for Wunderground)
        g = wind gust last 5 min in mph
        t = temperature in Fahrenheit, negative -1 to -99
        r = rain in inch/100 last hour
        p = rain in inch/100 last 24h
        P = rain in inch/100 since midnight
        h = humidity in %
        b = pressure in hPa/10
        */
        char buffer[15];              // buffer to store the string

        //LoRaString = CALLSIGN;
        //LoRaString += (">APRS:!");
        String APRSString = "";            // clear the string
        APRSString += LATITUDE;
        APRSString += ("/");
        APRSString += LONGITUDE;
        APRSString += ("_");  // separator

      #ifdef WITH_WIND
        // formatting wind datas like ".../...g..."
        int ws = int(round(windSpeed_avg2m * 0.62));      // speed i mph &conversion float to int after rounding
        int wg = int(round(windgustSpeed * 0.62));        // gust in mph & conversion float to int after rounding

        // creating the string
          sprintf(buffer, "%03d/%03dg%03d",
                              windDir_avg2m,
                              ws,
                              wg ); // 
        APRSString += buffer;                                   // push into beacon frame
      #else
        APRSString += (".../...g...");  // needed if no wind datas are sent
      #endif

        // temp, humidity & pressure string formatting
        float tempF = (tempC * 1.8) + 32;  // conversion Celsius to Fahrenheit

        #if defined(WITH_BME280) || defined(WITH_BME680)
        sprintf(buffer, "t%03dh%02db%05d",
                          int(tempF),
                          int(round(humi)),
                          int(round(press*10))
                          );
        #endif
        // BMP280 has no humidity sensor, so set to
        #if defined(WITH_BMP280)
              sprintf(buffer, "t%03dh..b%05d",
                    int(tempF),
                    int(round(press*10))
                    );
        #endif
        // SHT31 has no pressure sensor, so set to
        #if defined(WITH_SHT31)
        sprintf(buffer, "t%03dh%02db.....",
                          int(tempF),
                          int(round(humi))
                          );
        #endif
        APRSString += buffer;                                   // push into beacon frame

      // rain datas formatting
      #ifdef WITH_RAIN
        byte rain1hAPRS = int(round(rain1hmm * 3.937));               // convert mm to 100/inch
        unsigned int rain24hAPRS = int(round(rain24hmm * 3.937));   // convert mm to 100/inch
        sprintf(buffer, "r%03dp%03dP...",
                                    rain1hAPRS,
                                    rain24hAPRS );  // 
        APRSString += buffer;                                       // push into beacon frame
      #endif

      #ifdef SEND_BAT_INFO
        APRSString += "(Bat=" + String(batteryVoltage) + "V)";
      #endif
      // add COMMENT
      APRSString += " " + String(COMMENT);

      return APRSString;
  } // build_APRSbeacon END

  /**************************************************************************
   send all datas to the APRS-IS network
  **************************************************************************/
  void send2APRS_IS(String datas, bool status) {
    //String APRSString = datas;  
    //char login[60];

    String APRSISString =""; // contains datas for APRS-IS beacon
    
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "APRS-IS", "Connecting to APRS server...");
    
    int retr = 5;
    while (!client.connect(APRS_IS_SERVER, APRS_IS_SERVER_PORT) && (retr > 0)) {
      delay(50);
      --retr;
    }

    if (!client.connected()) {
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "APRS-IS", "Connection failed");
      client.stop();
      return;
    }
    else
    {
      //sprintf(login, "user %s pass %s to WX STATION TK5EP", APRS_IS_CALLSIGN, APRS_IS_PASSWD);
      String login = "user " + CALLSIGN + " pass " + APRS_IS_PASSWD;
      client.println(login);
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "APRS_IS", "Connected %s",login.c_str());
      delay(1000); //as recommended, delay between login and packet sending
      APRSISString += CALLSIGN;
      APRSISString += ">APEP01,TCPIP*";
      if (!status) APRSISString += ":=";  // datas packet if bool status = false
      else APRSISString += ":>";          // status packet if bool status = true
      APRSISString += datas;
      client.println(APRSISString); // send the datas to APRS-IS
      show_display("TX APRS-IS", "", APRSISString);
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "APRS_IS", "Sending : %s",APRSISString.c_str());
    }
  }

  /**************************************************************************
  send LoRa data via APRS
  bool status -> true send a status packet, false send a data packet
  ***************************************************************************/
  void send2APRS_LoRa(String datas, bool status) {
      
      String LoRaString = "";              // contains the LoRa beacon
      LoRaString = CALLSIGN;
      LoRaString += ">APEP01";
      #ifdef WITH_DIGIPEATING
        LoRaString += ",WIDE1-1";
      #endif
      if (!status) {
        LoRaString += (":!");    // this is for APRS LoRa data packet
      }
      else {
        LoRaString += (":>");    // this is for APRS LoRa status packet
      }

      LoRaString += datas;

      show_display("ON AIR", LoRaString,2000);
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "APRS-LoRa",  "Sending LoRa beacon %s" ,LoRaString.c_str());

      int state = radio.transmit("\x3c\xff\x01" +LoRaString);
      if (state == RADIOLIB_ERR_NONE) {
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "APRS-LoRa", "Sending success");
      }
      else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
        // the supplied packet was longer than 256 bytes
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "APRS-LoRa", "Packet too long !");
      }
      else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
        // timeout occurred while transmitting packet
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "APRS-LoRa", "LoRa timeout !");
      }
      else {
        // some other error occurred
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "APRS-LoRa", "LoRa transmission failed with code : %s",state);
      }
  }
} // end