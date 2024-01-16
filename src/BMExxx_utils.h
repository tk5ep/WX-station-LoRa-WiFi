#ifndef BME280_UTILS_H_
#define BME280_UTILS_H_

#include <Arduino.h>


namespace BME280_Utils {
        void read();
        void init();
        float seaLevelForAltitude(float altitude, float p0); 
}
#endif