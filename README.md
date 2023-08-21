# WX-station-LoRa-WiFi
An APRS LoRa weather station with a TTGO T3 module.

This is the description of a Weather station build with a TTGO T3 module that can :
- measure pressure, temperature, humidity, wind speed, wind direction, wind gust speed & direction, rain fall.
- transmit datas to APRS in LoRa, APRS-IS, Wunderground & MQTT via WiFi.
- display datas on the OLED screen.
- display a Web page server with data display.
- upgrade via OTA.

The pressure, temperature & humidity are measured with a BME280 multi-sensor on the i2C bus.
The wind direction and speed are measured in ModBus mode via RS485, allowing long distance measurements.
The rain fall is measured with a rain bucket gauge.

**Hardware :**
The used module is a  TTGO T3 v1.6.1 bought on Aliexpress at the LilyGp shop, i can only recommend.

The wind speed and direction sensors are a ModBus RS485 version that can be found (here on Aliexpress] (https://www.aliexpress.com/item/1005005500304078.html)
There are different models for RS485 sensors. Be aware that the library used here has been written by me for the above sensors.
The main difference is that the ModBus address is set by writing into a register, not sending a ModBus command.
