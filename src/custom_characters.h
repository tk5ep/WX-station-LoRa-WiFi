#ifndef CUSTOM_CHARACTERS_H_
#define CUSTOM_CHARACTERS_H_

#include <Arduino.h>

static const unsigned char bluetoothSymbol[] PROGMEM = {
    0b00001100, 0b00000000,
    0b00001111, 0b00000000,
    0b00001100, 0b11000000,
    0b11001100, 0b00110000,
    0b00111100, 0b11000000,
    0b00001111, 0b00000000,
    0b00001111, 0b00000000,
    0b00111100, 0b11000000,
    0b11001100, 0b00110000,
    0b00001100, 0b11000000,
    0b00001111, 0b00000000,
    0b00001100, 0b00000000,
    0b00000000, 0b00000000,
    0b00000000, 0b00000000
};

// custom image for broken RS485 link
const unsigned char nolink [] PROGMEM = {
	0x40, 0x00, 0x60, 0x78, 0x30, 0xcc, 0x18, 0x06, 0x0c, 0x06, 0x06, 0x06, 0x13, 0x8c, 0x31, 0xc8, 
	0x60, 0x60, 0x60, 0x30, 0x60, 0x18, 0x33, 0x8c, 0x1e, 0x06, 0x00, 0x02
};

#endif