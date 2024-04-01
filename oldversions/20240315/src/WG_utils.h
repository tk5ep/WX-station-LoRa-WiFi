#ifndef WG_UTILS_H_
#define WG_UTILS_H_

#include <Arduino.h>

namespace WG_Utils {
    void send();
    double DewPoint(double celsius, double humidity);
}
#endif