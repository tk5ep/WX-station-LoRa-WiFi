#ifndef APRS_UTILS_H_
#define APRS_UTILS_H_

#include <Arduino.h>


namespace APRS_Utils {
    void LORAsetup();
    String build_APRSbeacon();
    void send2APRS_LoRa(String datas, bool status=false);
    bool send2APRS_IS(String datas, bool status=false);
}
#endif
