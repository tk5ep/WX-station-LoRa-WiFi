#ifndef UTILS_H_
#define UTILS_H_

#include <Arduino.h>
#include "display.h"

namespace Utils {
    String delayToString(unsigned long time_ms);
    void PrintOLED();
    void CalcWind();
    void CalcRain();
    void IRAM_ATTR IncRain();
    String get_reset_reason(uint8_t reason);
    String lat2APRS (float ddmmm);
    String lng2APRS (float ddmmm);
    int scanI2Cdevice(void);
}

#endif