/********************************************************
 * functions for the LoRa_WX by TK5EP
********************************************************/
#ifndef utils_h
#define utils_h

#include <Arduino.h>
//#include <WString.h>
#endif

extern String delayToString(unsigned long time_ms);
extern double DewPoint(double celsius, double humidity);
String Deg2Compass(int deg);
String get_reset_reason(uint8_t reason);