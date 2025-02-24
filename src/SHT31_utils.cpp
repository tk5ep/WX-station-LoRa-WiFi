#include "SHT31_utils.h"
#include <Arduino.h>
#include <logger.h>
#include "settings.h"
#include "display.h"

extern logging::Logger  logger;
extern float humi;
extern float tempC;

SHT31 sht(SHT31_I2C);            // SHT31 init

namespace SHT31_Utils {
/***************************************
 SHT31 sensor 
***************************************/
void init() {
    Wire.begin();
    Wire.setClock(100000);
    sht.begin();
    uint16_t sht_status = sht.isConnected();
    #ifdef DEBUG_SHT
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "SHT31", "SHT31 status %s", String(sht_status));
    #endif
    if (!sht_status) {
        show_display("SHT31","","ERROR", "SENSOR NOT FOUND", "CORRECT THIS",5000);
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "SHT31", "Couldn't find SHT31");
    }
}

/***************************************
read the SHT31 sensor 
***************************************/
void read() {
    if (sht.isConnected()) { 
        sht.read();
        tempC = sht.getTemperature();
        humi  = sht.getHumidity();
        #ifdef DEBUG_SHT
            logger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "SHT31", "Reading SHT31 tempC : %sC humi : %s%%", String(tempC),String(humi));
        #endif
    }
    else {
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "SHT31", "No answer, setting all values to 0");
        tempC = 0;
        humi = 0;
    }      

  }
}
