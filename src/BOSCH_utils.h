#ifndef BOSCH_UTILS_H_
#define BOSCH_UTILS_H_

#include <Arduino.h>

namespace BOSCH_Utils {
        void read();
        void init();
        float seaLevelForAltitude(float altitude, float p0); 
}
#endif