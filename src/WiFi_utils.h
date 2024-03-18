#ifndef WIFI_UTILS_H_
#define WIFI_UTILS_H_

#include <Arduino.h>

namespace WiFi_Utils {
    void connect();
    String processor(const String& var);
}
#endif