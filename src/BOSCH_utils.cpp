#include "BOSCH_utils.h"
#include "settings.h"
#include "display.h"
#include <logger.h>
#include <Adafruit_Sensor.h>
#include <vector>

#if defined(WITH_BME280) || defined(WITH_BME680) || defined(WITH_BMP280)
    #ifdef WITH_BME280
      #include <Adafruit_BME280.h>
      Adafruit_BME280   bme;
    #endif
    #ifdef WITH_BME680
      #include <Adafruit_BME680.h>
      Adafruit_BME680 bme;
    #endif
    #ifdef WITH_BMP280
      #include <Adafruit_BMP280.h>
      Adafruit_BMP280 bme;
    #endif

    #define SEALEVELPRESSURE_HPA (1013.25)

    extern logging::Logger  logger;
    extern float press;
    extern float humi;
    extern float tempC;

    namespace BOSCH_Utils {
        /************************************
        Read BMExxx sensor 
        *************************************/
        void init() {
            // init BME
            bool bme_status = bme.begin(BME_I2C);  //address either 0x76 or 0x77
            #ifdef WITH_BME280
              //BME280 weather station settings corresponding to datasheet
              bme.setSampling(Adafruit_BME280::MODE_FORCED, // Force reading after delayTime
                              Adafruit_BME280::SAMPLING_X1, // Temperature sampling set to 1
                              Adafruit_BME280::SAMPLING_X1, // Pressure sampling set to 1
                              Adafruit_BME280::SAMPLING_X1, // Humidity sampling set to 1
                              Adafruit_BME280::FILTER_OFF   // Filter off - immediate 100% step response
                        );
            #endif

            #ifdef WITH_BMP280
              bme.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                              Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                              Adafruit_BMP280::SAMPLING_X1,    /* Pressure oversampling */
                              Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                              Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
            #endif

            #ifdef WITH_BME680
                  bme.setTemperatureOversampling(BME680_OS_1X);
                  bme.setHumidityOversampling(BME680_OS_1X);
                  bme.setPressureOversampling(BME680_OS_1X);
                  bme.setIIRFilterSize(BME680_FILTER_SIZE_0);
            #endif

            if (!bme_status) {
              logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "BOSCH", "No valid BOSCH sensor found !");
              show_display("BME sensor","","ERROR", "NOT FOUND", "CORRECT THIS",5000);
            }
            else {
              logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "BOSCH", "BOSCH sensor found");
            }
        }

        /************************************
        Read BOSCH sensor 
        *************************************/
        void read() {
            #if defined(WITH_BME280) || defined(WITH_BMP280)
                bme.takeForcedMeasurement(); // has no effect in normal mode
            #endif
            // only BME280, BME680 have humidity sensor
            #if defined(WITH_BME280) || defined(WITH_BME680)
              humi  = bme.readHumidity();            // humidity in %
              if (tempC > 100) {                    // Houston we've a problem ! BME has been disconnected or broken, restart to get at least a hand on the system with OTA
                logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "BME280", "No answer anymore, reboot!");
                ESP.restart();
              }
            #else
              humi = 0.0;  
            #endif
            // all BOSCH sensors have temperature reading, so we read it on all sensors
            tempC = bme.readTemperature();        // tempC in Centigrade
            // same for pressure            
            float p0 = (bme.readPressure() / 100);  // pressure in hPa without altitude correction
            #ifdef WITH_SEALEVELPRESSURE
                press = seaLevelForAltitude(ALTITUDE,p0);
            #else
              press = p0;
            #endif

            #ifdef DEBUG_BME
              logger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "BME280", "Reading BME280 press: %.02f hPa tempC: %.02f °C humi: %.02f %%",press,tempC,humi);
            #endif
        }

        /************************************************************************
         *   Calculates the pressure at sea level (in hPa) from the specified
         *   @param  altitude      Altitude in meters
         *   @param  p0            Measured atmospheric pressure in hPa
         *   @returns              the pressure at sea level (in hPa) from the specified altitude
         *************************************************************************/
        float seaLevelForAltitude(float altitude, float p0) {
          return p0 / pow(1.0 - (altitude / 44330.0), 5.255);
        }
} // END namespaces
#endif