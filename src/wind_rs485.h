/*
Test library to communicate via ModBus protocol with the wind sensors (anemometer and vane) and RS485 hardware
On a TTGO T3 module, serial port 1 uses GPIO12 & GPIO13 to communicate with sensors

The address of one device has to be modified first, both can't keep the same address on the bus. By default, they have the same
Connect ONLY the device to be changed on the bus, and uncomment the line "ModifyAdress(0x00,Address)"
0x00 is the broadcast address and Address the new address to be set, ex 0x01, modify this later field with the wished new address
Execute software and recomment the line
Cut power on device and restart.
*/

#ifndef WIND_RS485
#define WIND_RS485

#include <Arduino.h>


namespace wind_rs485 {
    void init();
    bool checkDevice(uint8_t);
    void addCRC(uint8_t *buf, int len);
    float readWindSpeed(uint8_t Address);
    int readWindDirection(uint8_t Address);
    boolean ModifyAddress(uint8_t Address1, uint8_t Address2);
    size_t readN(uint8_t *buf, size_t len);
    uint16_t CRC16_2(uint8_t *buf, int16_t len);
    void changeRS485address();
};
#endif // end of library