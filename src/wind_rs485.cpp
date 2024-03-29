/*
Test library to communicate via ModBus protocol with the wind sensors (anemometer and vane) and RS485 hardware
To be used with the FXJT-N01 and FSJT-N01 sensors like these : 
https://fr.aliexpress.com/item/1005005500304078.html
https://www.renkeer.com/

adapted from here :
https://github.com/DFRobotdl/RS485_Wind_Direction_Transmitter_V2

The address of one device has to be modified first, both can't keep the same address on the bus. By default, they have the same
Connect ONLY the device to be changed on the bus, and in settings.h file change the lines : 
bool ChangeAddress = false;" to "true
uint8_t AddressSpeedSensor = 0x01; <== here the speed sensor address
uint8_t AddressDirSensor = 0x02;   <== here the direction sensor address

Execute software and recomment the line
Cut power on device and restart.
*/

#include <Arduino.h>
#include "wind_rs485.h"
#include "settings.h"
#include "logger.h"
#include "display.h"

extern logging::Logger  logger;
extern HardwareSerial rs485Serial;
extern const uint8_t NewSensorAddress;

namespace wind_rs485 {

// init the serial communication
void init() {
  #define RXD 34                              // UART1 RXD pin
  #define TXD 4                               // UART1 TXD pin
  rs485Serial.begin(4800,SERIAL_8N1,RXD,TXD); // default speed in bauds
  logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RS485", "Init done");
}

/*
@brief check if the device returns right address
@param device address
@return Return true if correct
*/
bool checkDevice ( uint8_t Address) {
    //uint8_t Data[7] = {0}; //Store the original data packet returned by the sensor
    uint8_t COM[8] = {0x00, 0x04, 0x07, 0xD0, 0x00, 0x01, 0x00, 0x00}; // Command for reading address register
    boolean ret = false; 
    long curr = millis();
    long curr1 = curr;
    uint8_t ch = 0;
    COM[0] = Address; 
    addCRC(COM , 6); 
    rs485Serial.write(COM, 8); //Send the command

    while (!ret) {
      if (millis() - curr > 1000) {
        ret = false; 
        break;
      }
      if (millis() - curr1 > 100) {
          rs485Serial.write(COM, 8); //If the last command to read the wind directionissent for more than 100 milliseconds and the return command has not beenreceived, the command to read the wind direction will be re-sent
          curr1 = millis();
      }
      if (readN(&ch, 1) == 1) {
        if (ch == Address) {                  // returned device address
          if (readN(&ch, 1) == 1) {
             if (ch == 0x04) {                // returned command
              if (readN(&ch, 1) == 1) {
                if (ch == 0x02) {             // returned data length
                 if (readN(&ch, 1) == 1) {
                   if (ch == 00 ) {           // returned address MSB
                    if (readN(&ch, 1) == 1) {
                    if (ch == Address) {      // returned address LSB
                        ret = true ;          // if all has been correcty checked
                       }
                     }
                   }
                  }
                }
              }
            }
          }
        }
      }
    }
    return ret;
}

/*
@brief check all bytes in buffer within 1500 ms timeout
@param buf Packet for calculating the check value
@param len Check the length
@return Return 1 if correct
*/
size_t readN(uint8_t *buf, size_t len) {
    size_t offset = 0, left = len;
    int16_t Tineout = 1500;
    uint8_t *buffer = buf;
    long curr = millis();
    while (left) {
        if (rs485Serial.available()) {
            buffer[offset] = rs485Serial.read();
            offset++;
            left--;
        }
        if (millis() - curr > Tineout) {
            break;
        }
    }
    return offset;
}

/*
@brief Calculate CRC16_2 check value
@param buf Packet for calculating the check value
@param len Check the date length
@return Return a 16-bit check result */
uint16_t CRC16_2(uint8_t *buf, int16_t len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++)
    {
      crc ^= (uint16_t)buf[pos];
      for (int i = 8; i != 0; i--)
      {
        if ((crc & 0x0001) != 0)
        {
          crc >>= 1;
          crc^= 0xA001;
        }
        else
        {
          crc >>= 1;
        }
      }
    }
    crc = ((crc & 0x00ff) << 8) | ((crc & 0xff00) >> 8);
    return crc;
}

/*
@brief Add a CRC_16 check to the end of the packet
@param buf Packet that needs to add the check value
@param len Length of data that needs to add the check
@return None
*/
void addCRC(uint8_t *buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++)
    {
      crc ^= (uint16_t)buf[pos];
      for (int i = 8; i != 0; i--)
      {
        if ((crc & 0x0001) != 0)
        {
          crc >>= 1;
          crc^= 0xA001;
        }
        else
        {
          crc >>= 1;
        }
      }
    }
    buf[len] = crc % 0x100;           // LSB CRC
    buf[len + 1] = crc / 0x100;       // MSB CRC
}


/*
@brief Read the wind speed
@param Address The read device address
@return Wind speed unit m/s, return -1 for read timeout */
float readWindSpeed(uint8_t Address) {
    uint8_t Data[7] = {0}; //Store the original data packet returned by the sensor
    uint8_t COM[8] = {0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00}; //Command for reading wind speed, adjust for your sensor
    boolean ret = false; //Wind speed acquisition success flag
    float WindSpeed = 0;
    long curr = millis();
    long curr1 = curr;
    uint8_t ch = 0;
    COM[0] = Address; //Add the complete command package with reference to the communication protocol.
    addCRC(COM , 6); //Add CRC_16 check for reading wind speed commandpacket
    rs485Serial.write(COM, 8); //Send the command of reading the wind speed

    while (!ret) {
    if (millis() - curr > 1000) {
        WindSpeed = -1; //If the wind speed has not been read for more than 1000 milliseconds, it will be regarded as a timeout and return -1.
        break;
      }
      if (millis() - curr1 > 100) {
        rs485Serial.write(COM, 8); //If the last command to read the wind speed is sent for more than 100 milliseconds and the return command has not been received, the command to read the wind speed will be re-sent
        curr1 = millis();
      }
      if (readN(&ch, 1) == 1) {
         if (ch == Address) { //Read and judge the packet header.
         Data[0] = ch;
          if (readN(&ch, 1) == 1) {
            if (ch == 0x03) { //Read and judge the packet header.
              Data[1] = ch;
              if (readN(&ch, 1) == 1) {
                if (ch == 0x02) { //Read and judge the packet header.
                Data[2] = ch;
                  if (readN(&Data[3], 4) == 4) {
                      if (CRC16_2(Data, 5) == (Data[5] * 256 + Data[6])) { // Check CRC data packet
                        ret = true;
                        WindSpeed = (Data[3] * 256 + Data[4]) / 10.00; // Calculate the wind speed
                      }
                    }
                }
              }
            }
          }
        }
      }
    }
    return WindSpeed;
}

/*
@brief Read wind direction
@param Address The read device address.
@return Wind direction from 0 to 7, return -1 for read timeout. */
int readWindDirection(uint8_t Address) {
    uint8_t Data[7] = {0}; //Store the original data packet returned by the sensor
    uint8_t COM[8] = {0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00}; // Command for reading wind direction, adjust to your sensor
    boolean ret = false; //Wind direction acquisition success flag
    int WindDirection = 0;
    long curr = millis();
    long curr1 = curr;
    uint8_t ch = 0;
    COM[0] = Address; //Add the complete command package with reference to the communication protocol.
    addCRC(COM , 6); //Add CRC_16 check for reading wind direction commandpacket
    rs485Serial.write(COM, 8); //Send the command of reading the wind direction
    while (!ret) {
      if (millis() - curr > 1000) {
        WindDirection = -1; //If the wind direction has not been read for more than 1000 milliseconds, it will be regarded as a timeout and return -1.
        break;
      }
      if (millis() - curr1 > 100) {
          rs485Serial.write(COM, 8); //If the last command to read the wind directionissent for more than 100 milliseconds and the return command has not beenreceived, the command to read the wind direction will be re-sent
          curr1 = millis();
      }
      if (readN(&ch, 1) == 1) {
        if (ch == Address) { //Read and judge the packet header.
        Data[0] = ch;
          if (readN(&ch, 1) == 1) {
            if (ch == 0x03) { //Read and judge the packet header.
            Data[1] = ch;
              if (readN(&ch, 1) == 1) {
              if (ch == 0x02) { //Read and judge the packet header.
              Data[2] = ch;
                  if (readN(&Data[3], 4) == 4) {
                    if (CRC16_2(Data, 5) == (Data[5] * 256 + Data[6])) { //Checkdata packet
                      ret = true;
                      WindDirection = Data[3] * 256 + Data[4]; //Calculatethewind direction
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
    return WindDirection;
}


/*
@brief Modify the sensor device address by writing the x07D0 register
to change the address from 1 to 2, the command you should use is 01 06 07 D0 00 02 08 86
@param Address1 The address of the device before modification. Use the 0x00 address to set any address, after setting, you need to re-power on and restart
the module.
@param Address2 The modified address of the device, the range is 0x00~0xFF,
@return Returns true to indicate that the modification was successful, and returns false to indicate that the modification failed.*/
boolean ModifyAddress(uint8_t Address1, uint8_t Address2) {
    uint8_t ModifyAddressCOM[8] = {0x00, 0x06, 0x07, 0xD0, 0x00, 0x00, 0x00, 0x00};
    boolean ret = false;
    long curr = millis();
    long curr1 = curr;
    uint8_t ch = 0;
    ModifyAddressCOM[0] = Address1;
    ModifyAddressCOM[5] = Address2;
    addCRC(ModifyAddressCOM , 6);
    Serial.println(ModifyAddressCOM[0]);
    Serial.println(ModifyAddressCOM[5]);
    Serial.println(ModifyAddressCOM[6]);
    Serial.println(ModifyAddressCOM[7]);

    // TEST
    //uint8_t ModifyAddressCOM[8] = {0x01, 0x06, 0x07, 0xD0, 0x00, 0x02, 0x08, 0x86};
    //uint8_t ModifyAddressCOM[8] = {0x02, 0x06, 0x07, 0xD0, 0x00, 0x01, 0x48, 0xB4};
    rs485Serial.write(ModifyAddressCOM, 8); // sends the register modification request

    // only if the original address is not 0x00, check if echoed command is correct
    // because command adressed to 0x00 are not echoed
    if (Address1 != 0x00) {
            while (!ret) {
              if (millis() - curr > 1000) {
                break;
              }
              if (millis() - curr1 > 100) {
                rs485Serial.write(ModifyAddressCOM, 8);
                curr1 = millis();
              }
              if (readN(&ch, 1) == 1) {
                if (ch == Address1) {
                  if (readN(&ch, 1) == 1) {
                    if (ch == 0x06 ) { // 
                      if (readN(&ch, 1) == 1) {
                      if (ch == 0x07) {
                          if (readN(&ch, 1) == 1) {
                            if (ch == 0xD0) {
                            if (readN(&ch, 1) == 1) {
                              if (ch == 0x00) {
                              if (readN(&ch, 1) == 1) {
                                if (ch == Address2) {
                                    ret = true ;
                                }
                                else ret = false; // if not succeeded return false
                                }
                              }
                              }
                            }
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
    }
    else ret = true; // return true if Address1 is 0x00
    return ret;
} // modifyAddress END

/*******************************************
* change RS485 sensor address
********************************************/
void changeRS485address() {
        if (!wind_rs485::ModifyAddress(OldSensorAddress,NewSensorAddress)) {
            logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "RS485", "No communication with sensor");
            show_display("RS485","No answer","","Halted");
        }
        else {
            logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RS485", "Address change OK");
            logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RS485", "Release switch and repower the sensor !");
            show_display("RS485","Address changed","","to 0x" + String(NewSensorAddress,HEX));
        }

        for (;;); // loop forever
        }
} // namespace END