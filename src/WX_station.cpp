  /**************************************************************
* LoRA APRS WX mini station by TK5EP
* using new library, configuration file, WiFi
*

Hardware :
----------
Build around a TTGO T3 LoRa ESP32 module
BME280 on i2C pins SDA=IO21, SCL=IO22
RS485 on pins RXd IO34 TXd IO0
Rain sensor on pin IO14
RS485 programming switch on IO25

Principle:
----------
sends regularily weather datas via APRS beacon on LoRa 433.775 MHz. Via WiFi to APRS-IS, Wunderground or a MQTT broker at a defined period : TXPERIOD
can use HCP or static IP address
measures temperature,pressure, humidity every TXperiod
measures rain every second and stores every min and past hour
measures wind speed, direction every second, calculates average every 2 min
measures wind gust/dir and stores every min and tracks max for past 10 min

Updates :
---------
311223 Added parameters in settings_sample for daylight saving time. Reworked variables to save some memory. BME280 into forced mode for better measures in WX station as per datasheet.
291223 Changed from ASyncElegantOTA lib to ElegantOTA. Needs declaration in platformio.ini
281223 Added RS485 checks if sensors are alive. Report state via new MQTT topics. 
261223 Added APRS digipeating possibility
251223 Rearranging some routines. Added sealevel pressure correction. New WITH_SEALEVELPRESSURE parameter inn settings.
241223 Correcting some small bugs. Adding NTP, WiFi RSSI
161223 Rearranging functions. Adding SHT31 support. Modifying MQTTpublish() to add a systematical connection to broker.
101223 Modified Web server so it displays only the available datas. Preparing SHT31 sensor support
200823 Added OTA and Web page server.
160823 Switched from Arduino IDE to platformIO. added wind_rs485 library
070723 Added static/DHCP address choice.
060723 Added display of modes set at boot. Removed IF_WIFI.
030723 Added APRS-IS protocol. Modified other functions to be compatible.
300623 Added MQTT protocol. Small bugs corrected. 
260623 Removed light sensor. Rebuild of build_LoRaString().
250623 Removed interrupt handling for the anemometer and vane routine. Replaced with RS485 ModBus protocol wind sensors.
210623 Back to interrupts in lieu of digitalread(). Using native BME280 library
200623 Added Wunderground support
190623 Test with digitalread to get ride of the interrupt bug

**************************************************************/
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SHT31.h>                  // https://github.com/RobTillaart/SHT31
#include <Wire.h>
#include <LoRa.h>                   // by Sandeep Mistry 0.8.0
#include "wind_rs485.h"             // custom written RS485 library
#include "index.htm"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>           // by Nick O'Leary
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include "NTP.h"                    // github.com/sstaub/NTP.git
#include "settings_sample.h"         // config file

/********************************************************************
 _____              __ _                       _   _             
/  __ \            / _(_)                     | | (_)            
| /  \/ ___  _ __ | |_ _  __ _ _   _ _ __ __ _| |_ _  ___  _ __  
| |    / _ \| '_ \|  _| |/ _` | | | | '__/ _` | __| |/ _ \| '_ \ 
| \__/\ (_) | | | | | | | (_| | |_| | | | (_| | |_| | (_) | | | |
 \____/\___/|_| |_|_| |_|\__, |\__,_|_|  \__,_|\__|_|\___/|_| |_|
                          __/ |                                  
                         |___/                                   
********************************************************************/
String SOFTWARE_VERSION = "1.0" ;
String SOFTWARE_DATE = "2023.12.31";

// define pins for a TTGO T3 module
#define LORA_SCK 5                  // GPIO5    - SX1276 SCK
#define LORA_MISO 19                // GPIO19   - SX1276 MISO
#define LORA_MOSI 27                // GPIO27   - SX1276 MOSI
#define LORA_CS 18                  // GPIO18   - SX1276 CS ---> NSS
#define LORA_RST 23                 // GPIO14   - SX1276 RST
#define LORA_IRQ 26                 // GPIO26   - SX1276 IRQ ---->DIO0
const byte PIN_CHANGE_ADDRESS = 25; // connect this pin to 3V3 at boot to change RS485 address. Green LED lights to warn that it is the case
const byte interruptPinRain = 14;   // rain sensor pin
// pins 4 & 34 are used by UART0 hardwareserial for the RS485 bus
// pins 21 & 22 for the i2C bus

// define counters
unsigned long lastSecond = 0;
byte seconds;                       // When it hits 60, increase the current minute
byte seconds_2m;                    // Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes;                       // Keeps track of where we are in various arrays of data
byte minutes_10m;                   // Keeps track of where we are in wind gust/dir over last 10 minutes array of data
byte hours = 0;
byte TXcounter;                     // counter for the period betweeb LoRa transmissions
String upTime;
uint64_t microSecondsSinceBoot;
String Reset_Reason;                // String to hold the last RESET reason

bool FIRSTLOOP = true;             // to check if it is the first loop

WiFiClient client;
#ifdef WITH_MQTT
  PubSubClient mqttclient(client);
#endif

String LoRaString   = "";
String APRSString   = "";
String APRSISString = "";

float tempC, tempF, humi, press;
#ifdef WITH_BME280
  // BME280 globals
  bool bme_status = false;
  Adafruit_BME280 bme;   // BME280 init
#endif

#ifdef WITH_SHT31
  bool sht_status = false;
  SHT31 sht(SHT31_I2C);            // SHT31 init
#endif

// DISPLAY SSD1306
#define SSD1306_ADDRESS 0x3C
#define OLED_RST -1       // shared pin. pin 16 crashes
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
String line1 = "";
String line2 = "";
String line3 = "";
String line4 = "";
String line5 = "";

// custom image for broken RS485 link
const unsigned char nolink [] PROGMEM = {
	0x40, 0x00, 0x60, 0x78, 0x30, 0xcc, 0x18, 0x06, 0x0c, 0x06, 0x06, 0x06, 0x13, 0x8c, 0x31, 0xc8, 
	0x60, 0x60, 0x60, 0x30, 0x60, 0x18, 0x33, 0x8c, 0x1e, 0x06, 0x00, 0x02
};

// rain sensor
#ifdef WITH_RAIN
    volatile float rain1h[60];                                            //60 floating numbers to keep track of 60 minutes of rain
    float rain1hmm = 0;                                                   // rain during last 1 hour
    volatile float rain24hmm = 0;                                         //
    volatile unsigned int rainCount = 0, oldrainCount = 0;  // used in rain counter
#endif  // end if WITH_RAIN

// wind sensors
#ifdef WITH_WIND
  HardwareSerial rs485Serial(1);            // declare UART0
  bool RS485WindSpeedSensorTimeout = false; // flag to check if the wind sensor is answering
  bool RS485WindDirSensorTimeout   = false; // flag to check if the wind sensor is answering
  int currentDir                   = 0;     // instant wind direction ModBus value in 8 direction values 
  volatile int windDirAvg[120];             // array of 120 ints to keep track of 2 minute average
  int windDir_avg2m                = 0;     // 2 minute average wind direction in degrees 0-360
  float currentSpeedms             = 0;     // instant wind speed ModBus value in m/s
  float currentSpeedKmh            = 0;     // instant wind speed ModBus value in km/h
  volatile float windSpeedAvg[120];         // to keep track of last 120 wind speed measures
  float windSpeed_avg2m            = 0;     // 2 minute average wind speed in km/h calculate from above array
  float windgustSpeed              = 0;     // wind gust speed max value
  int windgustDir                  = 0;     // wind gust dir max value
  float windgust_10m[10];                   // array of 10 floats to keep track of 10 minute max gust in m/s
  int windgustDir_10m[10];                  // array of 10 float values to keep track of the wind gust direction
#endif

#ifdef WITH_WIFI
  // Create AsyncWebServer object on port 80
  AsyncWebServer server(80);
  // NTP
  WiFiUDP wifiUdp;
  NTP ntp(wifiUdp);
#endif

/*****************************
Functions declare
******************************/
void WIFIconnect();
void readBME();
void readSHT31();
void LORAsetup();
void Send2Wunder();
void send2APRS_LoRa();
void send2APRS_IS();
void build_APRSbeacon();
void PrintOLED();
String get_reset_reason(uint8_t reason);
String Deg2Compass(int deg);
double DewPoint(double celsius, double humidity);
String delayToString(unsigned long time_ms);
void IRAM_ATTR IncRain();
void CalcRain();
String processor(const String& var);
void CalcWind();
void changeRS485address();
void MQTTconnect();
void MQTTpublish();
void onOTAEnd(bool success);

/*****************************
 _____      _               
/  ___|    | |              
\ `--.  ___| |_ _   _ _ __  
 `--. \/ _ \ __| | | | '_ \ 
/\__/ /  __/ |_| |_| | |_) |
\____/ \___|\__|\__,_| .__/ 
                     | |    
                     |_|    
******************************/
void setup() {
  Serial.begin(115200);
  Serial.println(F("LoRa APRS & Wunderground & MQTT station by TK5EP."));

  //initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c)) {  // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;) ;  // Don't proceed, loop forever
  }
  
  Reset_Reason = get_reset_reason(esp_reset_reason());
  Serial.print("Last reset reason was : ");
  Serial.println(Reset_Reason);

  // define pin to change RS485 address.
  pinMode(PIN_CHANGE_ADDRESS,INPUT_PULLDOWN);

  #ifdef WITH_WIND
    wind_rs485::init();     // init the RS485 bus
   
    // Change address of wind sensor at startup
    // connect only one sensor on bus and close the switch wired on IO25. Green LED must light and press reset
    if (digitalRead(PIN_CHANGE_ADDRESS) == HIGH) {
        Serial.println("ModBus address change");
        changeRS485address();
    }
  #endif

  // Start LoRa module
  LORAsetup();

  #ifdef WITH_BME280
    // init BME280
    bme_status = bme.begin(BME280_I2C);  //address either 0x76 or 0x77
     //BME280 weather station settings corresponding to datasheet
    bme.setSampling(Adafruit_BME280::MODE_FORCED, // Force reading after delayTime
                  Adafruit_BME280::SAMPLING_X1, // Temperature sampling set to 1
                  Adafruit_BME280::SAMPLING_X1, // Pressure sampling set to 1
                  Adafruit_BME280::SAMPLING_X1, // Humidity sampling set to 1
                  Adafruit_BME280::FILTER_OFF   // Filter off - immediate 100% step response
                );

    if (!bme_status) {
      bme_status = true;
      Serial.println(F("No valid BME280 found !"));
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.setCursor(10, 0);
      display.print("No BME280");
      display.setCursor(10, 20);
      display.print("Correct");
      display.setCursor(10, 40);
      display.print("this");
      display.display();
      //for(;;); // Don't proceed, loop forever
      delay(10000);
    }
    else {
      Serial.println(F("BME280 found"));
    }
  #endif

  #ifdef WITH_SHT31
    Wire.begin();
    Wire.setClock(100000);
    sht.begin();
    uint16_t sht_status = sht.isConnected();
    #ifdef DEBUG_SHT
        Serial.print("SHT status : ");
        Serial.println(sht_status);
    #endif
    if (!sht_status) { 
        Serial.println("Couldn't find SHT31");
        display.clearDisplay();
        display.setTextColor(WHITE);
        display.setTextSize(2);
        display.setCursor(10, 0);
        display.print("No SHT31");
        display.setCursor(10, 20);
        display.print("Correct");
        display.setCursor(10, 40);
        display.print("this");
        display.display();
        delay (5000);
    }
  #endif

  // init rain sensor interrupt pin
  #ifdef WITH_RAIN
    pinMode(interruptPinRain, INPUT_PULLUP);  // rain sensor
    attachInterrupt(digitalPinToInterrupt(interruptPinRain), IncRain, RISING);
  #endif

  // boot info page on OLED
  line1 = "APRS LoRa";
  line2 = "WX station";
  String line3="Set:";          // build line 3
  #ifdef WITH_APRS_LORA
    line3+="LORA ";
  #endif
  #ifdef WITH_APRS_IS
      line3+="APRS-IS ";
  #endif
  #ifdef WITH_MQTT
      line3+="MQTT ";
  #endif
  #ifdef WITH_WUNDERGROUND
        line3+="WG";
  #endif
  line4 = "by TK5EP v" + SOFTWARE_DATE;
  line5 = "";

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(line1);
  display.setCursor(0, 20);
  display.print(line2);
  display.setTextSize(1);
  display.setCursor(0,40);
  display.print(line3);
  display.setCursor(0, 56);
  display.print(line4);
  display.display();
  delay(5000);

  #ifdef WITH_MQTT
    mqttclient.setServer(mqtt_broker, mqtt_port);
    mqttclient.setKeepAlive(300);
  #endif

  #ifdef WITH_WIFI
    WIFIconnect();    // connect to WiFi AP
    // NTP
    ntp.ruleDST(DSTzone, DSTweek, DSTwday, DSTmonth, DSTwday, DSToffset); 
    ntp.ruleSTD(STDzone, STDweek, STDwday, STDmonth, STDwday, STDoffset); 
    ntp.begin();
    // OTA
    ElegantOTA.begin(&server, OTA_username, OTA_password);
    ElegantOTA.onEnd(onOTAEnd);   // to report if OTA update has worked
  #endif

}  // setup() end


/*****************************
 _                       
| |                      
| |     ___   ___  _ __  
| |    / _ \ / _ \| '_ \ 
| |___| (_) | (_) | |_) |
\_____/\___/ \___/| .__/ 
                  | |    
                  |_|    
*****************************/
void loop() {
  ElegantOTA.loop();
  // this is the main counter that counts every second
  if (millis() - lastSecond >= 1000) {
    // increment the counter by 1 second, this is the base of all next counters (2min, 10 min, etc..)
    lastSecond += 1000;
    
    uint64_t microSecondsSinceBoot = esp_timer_get_time();  
    upTime = delayToString(microSecondsSinceBoot / 1000);
    //Serial.println(upTime);

    //counts every 2 min
    if (++seconds_2m > 119) {
      seconds_2m = 0;
    }

/*
        #ifdef WITH_MQTT
          mqttclient.loop();
           #ifdef DEBUG_MQTT
            Serial.println("MQTT loop");
          #endif  
        #endif

      #ifdef DEBUG_MQTT
        int mqttstate = mqttclient.state();
        bool mqttConnectState = mqttclient.connected();
        Serial.print("MQTT status state : ");
        Serial.println(mqttstate);
        Serial.print("MQTT connection state : ");
        Serial.println(mqttConnectState);
        if (!mqttConnectState) MQTTconnect();
      #endif
*/      
    // read wind sensors anc calculate wind infos
    #ifdef WITH_WIND
        RS485WindSpeedSensorTimeout = false;    // set the timeout flags
        RS485WindDirSensorTimeout   = false;
        // read wind speed and direction
        currentSpeedms  = wind_rs485::readWindSpeed(AddressSpeedSensor);    // read the wind speed, returns m/s or -1 if no answer from sensor
        delay(200);                                                         // needed to chain sensor readings
        currentDir      = wind_rs485::readWindDirection(AddressDirSensor);  // read the wind direction, returns degrees 0-360 or -1 if no answer from sensor
        // check if the ModBus sensors are answering
        if (currentSpeedms == -1)                                          // if we receive a timeout warning
        {
          Serial.println(F("Wind Speed ModBus timeout !"));
          currentSpeedKmh = 0;
          RS485WindSpeedSensorTimeout = true;                               // set the timeout flag
        }
        else currentSpeedKmh = currentSpeedms * 3.6;                       // convert into km/h
          
        if (currentDir == -1 )                                              // if we receive a timeout warning
        {
          Serial.println(F("Wind direction ModBus timeout !"));
          currentDir = 0 ;
          RS485WindDirSensorTimeout = true;
        }
        // compute all wind datas
        CalcWind();
    #endif

    if ((TXcounter == TXPERIOD - 5) && (ECOMODE == 2 )) display.ssd1306_command(SSD1306_DISPLAYON);  // reactivate display 5s before TXing
    
    // we send out datas every TXPERIOD and at boot
    if (++TXcounter == TXPERIOD || FIRSTLOOP) {       // if TX period has been reached and at boot
      TXcounter = 0;                                  // TXcounter reset
      //Serial.print("Wifi status : ");
      //Serial.println(WiFi.status());

        // read BME280 sensor
        #ifdef WITH_BME280
          if (bme_status) readBME();
        #endif

        #ifdef WITH_SHT31
        // if SHT31 connected read it
          if (sht.isConnected()) { 
            readSHT31();   
          }
          else{
            Serial.println ("SHT error");
            tempC = 0;
            humi = 0;
          }
        #endif

      if (FIRSTLOOP) {
        FIRSTLOOP = false;
        PrintOLED();      // display datas at startup before
        delay(3000);
      }

      build_APRSbeacon();         // build the LoRa beacon string
      #ifdef WITH_APRS_LORA
        send2APRS_LoRa();           // transmit the LoRa beacon
      #endif
      #ifdef WITH_WIFI
        if (WiFi.status() != 3) WIFIconnect();        // if not connected, reconnect to WiFi
        if (WiFi.status() == 3) {                     // if connected to WiFi, send datas
          Serial.printf("RSSI %d dBm\r\n",WiFi.RSSI());

          ntp.update();
          Serial.println(ntp.formattedTime("%A %d %B %Y %T")); // date & time from NTP

          #ifdef WITH_APRS_IS
            send2APRS_IS();                           // transmit the APRS IS beacon
          #endif
          #ifdef WITH_WUNDERGROUND
            Send2Wunder();
          #endif
          #ifdef WITH_MQTT                // transmit to MQTT broker
            MQTTpublish();                // publish in MQTT broker
          #endif
        }
      #endif
      // if ecomode than display OFF to save battery
      if (ECOMODE != 0 && !FIRSTLOOP)                  // if screensave is ON, wait after the 1st complete loop before screen OFF
        {
          display.ssd1306_command(SSD1306_DISPLAYOFF);
        }
    }  
    // update OLED display every second
    PrintOLED();
    
    // every minute do what has to be done
    if (++seconds > 59) {                     // when a minute is past
            seconds = 0;

            // here everything that has to be done every minute

            #ifdef WITH_WIND
                  // reset the stored values in array for next min
                  windgust_10m[minutes_10m+1] = 0;
                  windgustDir_10m[minutes_10m+1] = 0;
            #endif
            #ifdef WITH_RAIN
                  if (rainCount != oldrainCount) {  // did the raincounter increase ?
                    //if (rainCount == oldrainCount+2) rainCount--; // substrate 1 count to correct a bug in the ESP32 interrupt handling. Counts FALLING AND RISING
                    CalcRain();
                    oldrainCount = rainCount;  // set counter flag
                  }     
                  rainCount=0;                    // clear rain bucket counter
                  rain1h[minutes] = 0;            // every min, clear the old array value 
                  #if DEBUG_WIND
                      Serial.print(F("Rain 1h mm :"));
                      Serial.println(rain1hmm);
                      Serial.print(F("Rain 24h mm :"));
                      Serial.println(rain24hmm);
                  #endif
            #endif

            // counts every 10 minutes
            if (++minutes_10m > 9) {
              minutes_10m = 0;
            }

            // counts minutes and resets at hour
            if (++minutes > 59) {
              #ifdef WITH_RAN
                  minutes = 0;
                  rain1hmm = 0;                   // reset the last 60 min rain accumulation to make a new calc
              #endif
              // counts the hours for a full day
              if (++hours > 23) {
                  hours = 0;
                  #ifdef WITH_RAIN
                      rain24hmm = 0;      // reset daily rainfall
                  #endif
                }  
            }

            
    } // end minute count

    // every 10 seconds 
    if (seconds % 10 == 0) {

    }

    #ifdef DEBUG_TIME
        Serial.println(F("sec/2min/min/10min/TX"));
        Serial.print(seconds);
        Serial.print("/");
        Serial.print(seconds_2m);
        Serial.print(F("/"));
        Serial.print(minutes);
        Serial.print(F("/"));
        Serial.print(minutes_10m);
        Serial.print(F("/"));
        Serial.print(TXcounter);
        Serial.println();
    #endif
  }  // end if millis > 1000
}  // end LOOP
   ////////////////////////////////////////////////////////////////////////////
   // finally !


/*******************************
 _____                 _   _                 
|  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
| |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
|  _|| |_| | | | | (__| |_| | (_) | | | \__ \
|_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/ 

*******************************/

/**********************************
connect to WiFi
***********************************/
void WIFIconnect() {
 // init WiFi if needed
//#if defined WITH_APRS_IS || defined WITH_MQTT || defined WITH_WUNDERGROUND
#ifdef WITH_WIFI
  Serial.print(F("WiFi trying to connect"));
  line1 = "WiFi";
  line2 = "Trying to connect...";
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(line1);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print(line2);
  display.display();
  
  // is we use a static IP, not DHCP
  if (WITH_STATIC_IP)                     
    {
      if (WiFi.config(local_IP, gateway, subnet, primaryDNS))
        { Serial.println(F("STA configured")); }
      else 
        { Serial.println(F("STA Failed to configure")); } 
    }
  // Connect to WPA/WPA2 network.
  WiFi.begin(ssid, password);
  // try 20 WiFi connections
  int numberOfTries = 20;                 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (numberOfTries <= 0) {
      break; 
      }
    else { numberOfTries--; }
  }
  // if connection succeeded
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("\r\nWiFi connected IP address: "));
    Serial.println(WiFi.localIP());
    Serial.printf("RSSI %d dBm\r\n",WiFi.RSSI());

    // display on OLED the connection process
    char buffer[20];
    sprintf(buffer,"IP : %u.%u.%u.%u", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
    line1= "WiFi";
    line2 = "Wifi connected to :";
    line3 = ssid;
    line4 = buffer;
    line5 = "RSSI : " + String(WiFi.RSSI()) + " dBm";
  } else {
    Serial.println("\r\nWiFi failed");
    WiFi.disconnect();

    line1 = "WiFi";
    line2 = "WiFi failed !";
    line3 = "";
    line5 = "";
    if (WiFi.status() == WL_NO_SSID_AVAIL) {
      Serial.println(F("SSID not found"));
      line4 = "SSID not found";
    }
  }
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(line1);
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.print(line2);
    display.setCursor(0, 32);
    display.print(line3);
    display.setCursor(0, 44);
    display.print(line4);
    display.setCursor(0, 56);
    display.print(line5);
    display.display();
    delay(4000);

    // Handle Web Server
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/html", index_html, processor);
    });
    server.begin();
#endif 
}

#ifdef WITH_BME280
/************************************
Read BME280 sensor 
*************************************/
void readBME() {
  bme.takeForcedMeasurement(); // has no effect in normal mode
  // get BME280 values
  float p0 = (bme.readPressure() / 100);  // pressure in hPa without altitude correction
  #ifdef WITH_SEALEVELPRESSURE
    press = bme.seaLevelForAltitude(ALTITUDE,p0);   // pressure with altitude correction
    #else
    press = p0;
  #endif

  tempC = bme.readTemperature();        // tempC in Centigrade
  humi = bme.readHumidity();            // humidity in %
  if (tempC > 100) {                    // Houston we've a problem ! BME has been disconnected or broken, restart to get at least a hand on the system with OTA
    ESP.restart();
  }

  #ifdef DEBUG_BME
    Serial.print(F("reading BME280 "));
    Serial.print(F("press :"));
    Serial.print(press);
    Serial.print(F("  tempC :"));
    Serial.print(tempC);
    Serial.print(F("  humi :"));
    Serial.println(humi);
  #endif
}
#endif

/***************************************
read the SHT31 sensor 
***************************************/
#ifdef WITH_SHT31
void readSHT31() {
      sht.read();
      tempC = sht.getTemperature();
      humi  = sht.getHumidity();
      #ifdef DEBUG_SHT
        Serial.print("reading SHT31");
        Serial.print(F("  tempC :"));
        Serial.print(tempC);
        Serial.print(F("  humi :"));
        Serial.println(humi);
      #endif
  }
#endif

/*****************************************************
 function : dewpoint calculations https://gist.github.com/Mausy5043/4179a715d616e6ad8a4eababee7e0281
 input : double celcius, double humidity
 returns : double dewpoint in °C
*****************************************************/
double DewPoint(double celsius, double humidity) {
  double RATIO = 373.15 / (273.15 + celsius); 
  double SUM = -7.90298 * (RATIO - 1);
  SUM += 5.02808 * log10(RATIO);
  SUM += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / RATIO))) - 1);
  SUM += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1);
  SUM += log10(1013.246);
  double VP = pow(10, SUM - 3) * humidity;
  double T = log(VP / 0.61078);  // temp var
  return (241.88 * T) / (17.558 - T);
}

/***************************************************
 function : converts degrees 0-360 into compass direction by 22.5 sectors ex : N = 348.75 to 11.25, NNE = 11.25 to 33.75
 input : int degrees 0-360
 returns : String cardinal direction "N"
*****************************************************/
String Deg2Compass(int deg) {
    String dir[17]={"N","NNE","NE","ENE","E","ESE","SE","SSE","S","SSW","SW","WSW","W","WNW","NW","NNW","N"};
    int d = int((deg + 11.25) /22.5);
    return dir[d];
}

/***************************************************
 function : get the ESP32 reset reason
 input : uint8_t answer of esp_reset_reason() 
 returns : String readable reset reason
***************************************************/
String get_reset_reason(uint8_t reason)
{
  String ret ="";
  switch ( reason)
  {
    // https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/reset_reason.html
    case 1 : ret = "Vbat power ON";break;          /**<1, Vbat power on reset*/
    case 3 : ret = "Software";break;               /**<3, Software reset digital core*/
    case 4 : ret = "Watch dog";break;             /**<4, Legacy watch dog reset digital core*/
    case 5 : ret = "Deep Sleep";break;              /**<5, Deep Sleep reset digital core*/
    case 6 : ret = "Reset by SLC module";break;             /**<6, Reset by SLC module, reset digital core*/
    case 7 : ret = "Timer Group0 Watch dog";break;       /**<7, Timer Group0 Watch dog reset digital core*/
    case 8 : ret = "Timer Group1 Watch dog";break;       /**<8, Timer Group1 Watch dog reset digital core*/
    case 9 : ret = "RTC Watch dog";break;          /**<9, RTC Watch dog Reset digital core*/
    case 10 : ret = "Intrusion";break;       /**<10, Instrusion tested to reset CPU*/
    case 11 : ret = "Time Group";break;       /**<11, Time Group reset CPU*/
    case 12 : ret = "Software reset CPU";break;          /**<12, Software reset CPU*/
    case 13 : ret = "RTC Watch dog";break;      /**<13, RTC Watch dog Reset CPU*/
    case 14 : ret = "EXT_CPU_RESET";break;         /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : ret = "Unstable VDD voltage";break;/**<15, Reset when the vdd voltage is not stable*/
    case 16 : ret = "RTCWDT_RTC_RESET";break;      /**<16, RTC Watch dog reset digital core and rtc module*/
    default : ret = "NO_MEAN";
  }
  return ret;
}

/***********************************************
 convert time in ms to day hour min sec
***********************************************/
String delayToString(unsigned long time_ms) {
	char buf[64];
	String s;

    unsigned long seconds = time_ms / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    unsigned long days = hours / 24;
  
    seconds %= 60;
    minutes %= 60;
    hours %= 24;

    sprintf_P(buf, PSTR("%03dd:%02dh:%02dm:%02ds"),days,hours,minutes,seconds );
    s = buf;
	return s;
}

/**************************************************************************
 upload to Wunderground
***************************************************************************
https://weatherstation.wunderground.com/weatherstation/updateweatherstation.php?ID=XXXXX&PASSWORD=YYYYYYY&dateutc=now&humidity=59&action=updateraw

datas format explained : https://support.weather.com/s/article/PWS-Upload-Protocol?language=en_US
action [action=updateraw] -- always supply this parameter to indicate you are making a weather observation upload
ID [ID as registered by wunderground.com]
PASSWORD [Station Key registered with this PWS ID, case sensitive]
dateutc - [YYYY-MM-DD HH:MM:SS (mysql format)] In Universal Coordinated Time (UTC) Not local time
winddir - [0-360 instantaneous wind direction]
windspeedmph - [mph instantaneous wind speed]
windgustmph - [mph current wind gust, using software specific time period]
windgustdir - [0-360 using software specific time period]
windspdmph_avg2m  - [mph 2 minute average wind speed mph]
winddir_avg2m - [0-360 2 minute average wind direction]
windgustmph_10m - [mph past 10 minutes wind gust mph ]
windgustdir_10m - [0-360 past 10 minutes wind gust direction]
humidity - [% outdoor humidity 0-100%]
dewptf- [F outdoor dewpoint F]
tempf - [F outdoor temperature]
* for extra outdoor sensors use temp2f, temp3f, and so on
rainin - [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
dailyrainin - [rain inches so far today in local time]
baromin - [barometric pressure inches]
*/
#ifdef WITH_WUNDERGROUND
void Send2Wunder() {
      // connect to wunderground
      if (!client.connect(WG_server, 80)) {
        Serial.println(F("Send2Wunder Failed"));
        return;
      } else {
        Serial.println(F("Connected. WeatherUnderground page updating...."));
      }

      float dewptf = (DewPoint(tempC, humi) * 1.8) + 32;  // compute dewpoint and convert Celsius to Fahrenheit

      String url = "/weatherstation/updateweatherstation.php?ID=";
      url += WG_ID;
      url += "&PASSWORD=";
      url += WG_PWD;
      url += "&dateutc=now&tempf=";
      url += tempF;
      url += "&humidity=";
      url += humi;
      url += "&dewptf=";
      url += dewptf;
      url += "&baromin=";
      url += (press * 0.02953f);  // 1 hPa = 0.02953 inHg
    #ifdef WITH_RAIN
      url += "&rainin=";
      url += rain1hmm / 25.4;  // mm to inches
      url += "&dailyrainin=";
      url += rain24hmm / 25.4;
    #endif
    #ifdef WITH_WIND
      url += "&windspeedmph=";
      url += (windSpeed_avg2m * 0.62);
      url += "&winddir=";
      url += windDir_avg2m;
      url += "&windgustmph=";
      url += (windgustSpeed * 0.62);
      url += "&windgustdir=";
      url += windgustDir;
    #endif
      //url += "&softwaretype=";
      //url += SOFT_VER;
      url += "&action=updateraw";

      Serial.println("Requesting :" + url);
      client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: weatherstation.wunderground.com\r\n" + "User-Agent: G6EJDFailureDetectionFunction\r\n" + "Connection: close\r\n\r\n");

    #ifdef DEBUG_WG
      delay(2000);  // let the time to server to answer
      //print server reply
      Serial.print(F("server reply:"));
      while (client.available()) {
        String line = client.readStringUntil('\r');
        Serial.print(line);
      }
    #endif
      client.stop();
}
#endif

/**************************************************************************
 show WX data on OLED display
**************************************************************************/
void PrintOLED() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(CALLSIGN);

    
  display.setTextSize(1);
  #ifdef WITH_BME280          // display pressure only if available
    display.setCursor(0, 20);
    display.print("PRES:");
    display.print(press,1);  //in hpa
    display.print(" hPa");
  #endif    
  display.setCursor(110, 20);
  char buffer[3];
  sprintf(buffer, "%03d", TXPERIOD - TXcounter);  // display countdown until beacon
  display.print(buffer);
  
  display.setCursor(0, 32);
  display.print("TEMP:");
  display.print(tempC, 1);
  display.println("C");
  display.setCursor(0, 44);
  display.print("HUMI:");
  display.print(humi, 0);
  display.println("%");

#ifdef WITH_RAIN
  display.setCursor(55, 44);
  display.print("RAIN:");
  display.print(rain1hmm);
  display.println("mm");
#endif
#ifdef WITH_WIND
  // if RS485 linkbroken, display a warning on OLED
    if (RS485WindSpeedSensorTimeout || RS485WindDirSensorTimeout) {
    display.drawBitmap(110, 0,  nolink, 16, 14, WHITE);
  }
  display.setCursor(0, 56);
  display.print("WIND:");
  display.print (windSpeed_avg2m,1);     // in km/h
  display.print(" Kmh");

  if (DISPLAY_CARDINAL) {               // display conversion degrees to compass directions
    display.setCursor(110, 56);
    display.print(Deg2Compass(windDir_avg2m));
  }
  else {
    display.setCursor(103, 56);
    display.print(windDir_avg2m);       
    display.setCursor(122, 56);
    display.print((char)247);           // ° degrees symbol
  }
#endif
  display.display();
}

/**************************************************************************
 send all datas to the APRS-IS network
**************************************************************************/
void send2APRS_IS() {
  char login[60];
  //char APRSISString[150];
  unsigned int len;
  APRSISString ="";                         // clear the string

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0, 30);
  display.print(F("TX APRS-IS"));
  //display.setTextSize(1);
  //display.setCursor(0, 20);
  //display.print(APRSString);
  display.display();

  Serial.print(F("Connecting to APRS server..."));
  int retr = 20;
  while (!client.connect(APRS_IS_SERVER, APRS_IS_SERVER_PORT) && (retr > 0)) {
    delay(50);
    --retr;
  }

  if (!client.connected()) {
    Serial.println(F("connection failed"));
    client.stop();
    return;
  }
  else
  {
    Serial.println(F("done"));
    sprintf(login, "user %s pass %s vers WX STATION TK5EP", APRS_IS_CALLSIGN, APRS_IS_PASSWD);
    client.println(login);
    Serial.println(login);
    delay(1000); //as recommended, delay between login and packet sending
    APRSISString += CALLSIGN;
    APRSISString += ">APWXEP,TCPIP*:=";
    APRSISString += APRSString;
    client.println(APRSISString);
    Serial.println(APRSISString);
  }
}

/**************************************************************************
send LoRa data via APRS
***************************************************************************/
void send2APRS_LoRa() {
  LoRaString = "";              // clear the string
  LoRaString = CALLSIGN;
  LoRaString += ">APWXEP";
  #ifdef WITH_DIGIPEATING
    LoRaString += ",WIDE1-1";
  #endif
  LoRaString += (":!");    // this is for APRS LoRa
  LoRaString += APRSString;

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(30, 0);
  display.print("ON AIR");
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print(LoRaString);
  display.display();

  Serial.println(F("Sending LoRa beacon."));
    // send packet
  LoRa.beginPacket();
  LoRa.write('<');   // next 3 lines are preamble
  LoRa.write(0xFF);  // LoRa packet. APRS = 0xFF  0xF8 = ?
  LoRa.write(0x01);
  LoRa.print(LoRaString);  // send out beacon content
  LoRa.endPacket();
  
  // check LoRaString on serial port
  Serial.print(F("LoRa String = "));
  Serial.println(LoRaString);
}

/**************************************************************************
    build the APRS beacon string 
***************************************************************************/
void build_APRSbeacon() {
      /* build string for LoRa APRS as per APRS protocol reference 
      ex !4112.28N/08403.57W_045/086g097t026r002p007P006h02b03241wDvs
      c = wind direction in degrees
      s = wind speed over 1 min in mph (we send over 2min like for Wunderground)
      g = wind gust last 5 min in mph
      t = temperature in Fahrenheit, negative -1 to -99
      r = rain in inch/100 last hour
      p = rain in inch/100 last 24h
      P = rain in inch/100 since midnight
      h = humidity in %
      b = pressure in hPa/10
      */
      char buffer[15];              // buffer to store the string
      //LoRaString = CALLSIGN;
      //LoRaString += (">APRS:!");
      APRSString = "";            // clear the string
      APRSString += LATITUDE;
      APRSString += ("/");
      APRSString += LONGITUDE;
      APRSString += ("_");  // separator

    #ifdef WITH_WIND
      // formatting wind datas like ".../...g..."
      int ws = int(round(windSpeed_avg2m * 0.62));      // speed i mph &conversion float to int after rounding
      int wg = int(round(windgustSpeed * 0.62));        // gust in mph & conversion float to int after rounding

      // creating the string
        sprintf(buffer, "%03d/%03dg%03d",
                            windDir_avg2m,
                            ws,
                            wg ); // 
      APRSString += buffer;                                   // push into beacon frame
    #else
      APRSString += (".../...g...");  // needed if no wind datas are sent
    #endif

      // temp, humidity & pressure string formatting
      tempF = (tempC * 1.8) + 32;  // conversion Celsius to Fahrenheit
      sprintf(buffer, "t%03dh%02db%05d",
                        int(tempF),
                        int(round(humi)),
                        int(round(press*10))); // 
      APRSString += buffer;                                   // push into beacon frame

    // rain datas formatting
    #ifdef WITH_RAIN
      byte rain1hAPRS = int(round(rain1hmm * 3.937));               // convert mm to 100/inch
      unsigned int rain24hAPRS = int(round(rain24hmm * 3.937));   // convert mm to 100/inch
      sprintf(buffer, "r%03dp%03dP...",
                                  rain1hAPRS,
                                  rain24hAPRS );  // 
      APRSString += buffer;                                       // push into beacon frame
    #endif

      //LoRaString += (" ");       // add a blank to clear the comment is not set
      // add COMMENT
    APRSString += (COMMENT);
} // build_APRSbeacon END

/**************************************************************************
 setup LORA module
***************************************************************************/
void LORAsetup() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  if (!LoRa.begin(TXFREQUENCY)) {
    Serial.println(F("Starting LoRa failed !"));
    while (true) {
    }
  }
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125000);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.setTxPower(TXPOWER);                 // set power in dBm
  Serial.println(F("LoRa init done."));
}

/***************************************
// counts numbers of rain buckets, uses soft debouncing
****************************************/
#ifdef WITH_RAIN

    void IRAM_ATTR IncRain() {
    volatile unsigned long rainlast;
    if (millis() - rainlast > RAINDEBOUNCE)  // suppress incoming bounces coming in before set debounce time
      {
        rainCount++;          // increment rain counter
        rainlast = millis();  // resets debounce time
      }
    }
#endif

/***************************************
    // calculates daily rain fall
****************************************/
#ifdef WITH_RAIN
    void CalcRain() { 
        rain24hmm       += rainCount * rainBucketCont;  // daily rainfall
        rain1h[minutes] += rainCount * rainBucketCont;  // rain quantity during last minute
        
        for (int i = 0; i < 60; i++) {   // calc total rainfall for last sliding 60 min (not last hour)
          rain1hmm += rain1h[i];
        }
        #ifdef DEBUG_RAIN
          Serial.print(F("Raincount :"));
          Serial.println(rainCount);
          Serial.print(F("Minute :"));
          Serial.println(minutes);
          Serial.print(F("Rain min :"));
          Serial.println(rain1h[minutes]);
        #endif
      }
#endif

/******************************************
 * processor for the Web page 
*******************************************/
String processor(const String& var){
  //Serial.println(var);
  if (var == "RESET")       return String(Reset_Reason);
  if (var == "CALLSIGN")    return String(CALLSIGN);
  if (var == "FREQUENCY")   { float TXFREQMHZ=TXFREQUENCY/1000000.000; return String(TXFREQMHZ,3);}
  if (var == "POWER")       return String(TXPOWER);  
  if (var == "VERSION")     return String(SOFTWARE_VERSION);
  if (var == "VERSION_DATE") return String(SOFTWARE_DATE);

  if (var == "TEMPERATURE") return "<div class=\"card\"><p><i class=\"fas fa-thermometer-half\"  style=\"color:#059e8a;\"></i> TEMPERATURE</p><p><span class=\"reading\"><span id=\"temp_html\">" +  String(tempC) + "</span> &deg;C</span></p></div>";
  if (var == "HUMIDITY")    return "<div class=\"card\"><p><i class=\"fas fa-tint\"              style=\"color:#00add6;\"></i> HUMIDITY</p><p><span class=\"reading\"><span id=\"hum_html\">" + String(humi) +    "</span> &percnt;</span></p></div>";
  #ifdef WITH_BME280 // if BME280 not SHT31, so display pressure value
    if (var == "PRESSURE")    return "<div class=\"card\"><p><i class=\"fas fa-angle-double-down\" style=\"color:#e1e437;\"></i> PRESSURE</p><p><span class=\"reading\"><span id=\"press_html\">" + String(press) + "</span> hPa</span></p></div>";
  #endif  
  
  #ifdef WITH_WIND
      if (var == "WINDSPEED") return "<div class=\"card\"><p><i class=\"fas fa-flag\" style=\"color:#059e8a;\"></i> WIND SPEED</p><p><span class=\"reading\"><span id=\"windspeed_html\">" + String(windSpeed_avg2m) + "</span> km/h</span></p></div>";
      if (var == "WINDDIR")   return "<div class=\"card\"><p><i class=\"fas fa-location-arrow\" style=\"color:#00add6;\"></i> WIND DIR</p><p><span class=\"reading\"><span id=\"winddir_html\">" + String(windDir_avg2m) + "</span> &deg;</span></p></div>";
      if (var == "GUSTSPEED") return "<div class=\"card\"><p><i class=\"fas fa-flag\" style=\"color:#FF0000;\"></i> GUST SPEED</p><p><span class=\"reading\"><span id=\"gustspeed_html\">" + String(windgustSpeed) + "</span> km/h</span></p></div>";
      if (var == "GUSTDIR")   return "<div class=\"card\"><p><i class=\"fas fa-location-arrow\" style=\"color:#FF0000;\"></i> GUST DIR</p><p><span class=\"reading\"><span id=\"gustdir_html\">" + String(windgustDir) + "</span> &deg;</span></p></div>";
  #endif
 
  #ifdef WITH_RAIN
      if (var == "RAIN1H")    return "<div class=\"card\"><p><i class=\"fas fa-cloud-rain\" style=\"color:#059e8a;\"></i> RAIN 1h </p><p><span class=\"reading\"><span id=\"rain1h_html\">"  + String(rain1hmm)  + "</span> mm</span></p></div>";
      if (var == "RAIN24H")   return "<div class=\"card\"><p><i class=\"fas fa-cloud-rain\" style=\"color:#00add6;\"></i> RAIN 24h</p><p><span class=\"reading\"><span id=\"rain24h_html\">" + String(rain24hmm) + "</span> mm</span></p></div>";
  #endif

  // display some parameters
      if (var == "TXPERIOD") return String(TXPERIOD);
      if (var == "UPTIME")   return String(upTime);
  #if defined WITH_APRS_LORA
      if (var == "WITHAPRS") return "<i class=\"fas fa-check\" style=\"color:#059e8a;\"></i>";
    #else
      if (var == "WITHAPRS") return "<i class=\"fas fa-times\" style=\"color:#FF0000;\"";
  #endif
  #if defined WITH_MQTT
      if (var == "WITHMQTT") return "<i class=\"fas fa-check\" style=\"color:#059e8a;\"></i>";
    #else
    if (var == "WITHMQTT") return "<i class=\"fas fa-times\" style=\"color:#FF0000;\"";
  #endif
  #if defined WITH_APRS_IS
      if (var == "WITHAPRSIS") return "<i class=\"fas fa-check\" style=\"color:#059e8a;\"></i>";
    #else
      if (var == "WITHAPRSIS") return "<i class=\"fas fa-times\" style=\"color:#FF0000;\"";
  #endif
  #if defined WITH_WUNDERGROUND
      if (var == "WITHWG") return "<i class=\"fas fa-check\" style=\"color:#059e8a;\"></i>";
    #else
      if (var == "WITHWG") return "<i class=\"fas fa-times\" style=\"color:#FF0000;\"></i>";
  #endif
  #ifdef WITH_WIFI
      if (var == "WIFI") return "<tr><td>WiFi</td><td colspan=\"2\">SSID : " + String(WiFi.SSID()) +                  "</td><td colspan=\"2\">RSSI : " + String(WiFi.RSSI()) + "dBm</td></tr>";
      if (var == "NTP") return "<tr><td>NTP</td>  <td colspan=\"2\">" + String(ntp.formattedTime("%d %B %Y")) + "</td><td colspan=\"2\">Time : "    + String(ntp.formattedTime("%T")) + "</td></tr>";
  #endif
  return String();
}

#ifdef WITH_WIND
  // *****************************************************
  // make all wind calculations
  // part of code found here : https://learn.sparkfun.com/tutorials/arduino-weather-shield-hookup-guide-v12/all#example-firmware---weather-station
  // *****************************************************
  void CalcWind() {
    // adds current 1s reading in float km/h into 2 min array
    windSpeedAvg[seconds_2m] = currentSpeedKmh;  
    // calculates the average wind speed from last 120 measures (2 min)
    for (int i = 0; i < 120; i++) {
      windSpeed_avg2m += windSpeedAvg[i];
    }
    windSpeed_avg2m /= 120.0;        // 2 min average wind speed

    // calculate wind direction
    //
    currentDir = currentDir * 45;               // transforms 0-7 values from sensor to 0-360 degrees

    
    windDirAvg[seconds_2m] = currentDir;
    // calculates the average wind dir from last 120 measures (2 min)
    for (int i = 0; i < 120; i++) {
      windDir_avg2m += windDirAvg[i];
    }
    windDir_avg2m /= 120.0;        // 2 min average wind speed
    /*
    // compute average wind direction
    // Based on: http://abelian.org/vlf/bearings.html
    // Based on: http://stackoverflow.com/questions/1813483/averaging-angles-again
    long sum = windDirAvg[0];             // 
    int D = windDirAvg[0];
    for(int i = 1 ; i < 120 ; i++)
    {
        int delta = windDirAvg[i] - D;

        if(delta < -180)
            D += delta + 360;
        else if(delta > 180)
            D += delta - 360;
        else
            D += delta;
        sum += D;
    }
    windDir_avg2m = sum /120;                       // calculate mean avg wind dir 0-360
    if(windDir_avg2m >= 360) windDir_avg2m -= 360;  // 
    if(windDir_avg2m < 0) windDir_avg2m += 360;
*/

    // do we have a wind gust for the minute ?
    if (currentSpeedKmh > windgust_10m[minutes_10m]) {
      windgust_10m[minutes_10m] = currentSpeedKmh;            // if yes store new values for speed into 10 floats array
      windgustDir_10m[minutes_10m] = currentDir;              // same for gust direction
    }
    windgustSpeed = 0;
    windgustDir = 0;
    for (int i = 0; i < 10; i++) {
        //windgustSpeed = max(windgust_10m[i],windgustSpeed);  // get the max values from 10 last
        if (windgust_10m[i] >= windgustSpeed) {
            windgustSpeed = windgust_10m[i];                  // max speed
            windgustDir = windgustDir_10m[i];                 // corresponding dir
        }
    }

  #ifdef DEBUG_WIND
    Serial.print(F("currentSpeed km/h : "));
    Serial.println(currentSpeedKmh);
    Serial.print(F("windSpeed_avg2m : "));
    Serial.println(windSpeed_avg2m);
    Serial.print(F("Gust speed :"));
    Serial.println(windgustSpeed);
    Serial.print(F("Gust dir :"));
    Serial.println(windgustDir);
    Serial.print(F("currentDir : "));
    Serial.println(currentDir);
    Serial.print(F("windDir_avg2m : "));
    Serial.println(windDir_avg2m);
    Serial.println();
  #endif
  }
#endif

/*******************************************
* change RS485 sensor address
********************************************/
void changeRS485address() {
        if (!wind_rs485::ModifyAddress(0x01,NewSensorAddress)) {
            Serial.println("No communication with sensor");
            line1 = "RS485";
            line2 = "No comm !";
            line3 = "Halted";
        }
        else {
            Serial.println("Address change OK");
            Serial.println("Release switch and repower the sensor !");
            line1 = "RS485";
            line2 = "changed";
            line3 = "to 0x" + String(NewSensorAddress,HEX);
        }
            display.clearDisplay();
            display.setTextColor(WHITE);
            display.setTextSize(2);
            display.setCursor(1, 10);
            display.print(line1);
            display.setCursor(1, 30);
            display.print(line2);
            display.setCursor(1, 50);
            display.print(line3);
            display.display();
        for (;;); // loop forever
        }

/**************************************************************************
 * connecting to MQTT brocker
 **************************************************************************/
#ifdef WITH_MQTT
void MQTTconnect(){
    uint8_t retries = 5;                             // number of connection tries
    while (!mqttclient.connected()) {
        String client_id = "WX-station";
        //client_id += String(WiFi.macAddress());
        Serial.printf("Client %s connects MQTT broker %s\r\n", client_id.c_str(),mqtt_broker);
        if (mqttclient.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.printf("%s brocker connected\r\n",mqtt_broker);
        } else {
            Serial.print("failed with state ");
            Serial.println(mqttclient.state());
            delay(1000);
        }
        retries--;

        if (retries == 0) {
        break;
        //while (1); // if we want to stop
        Serial.println(F("Connection to broker failed !"));
        }
    }
}

/**************************************************************************
 * publishing on MQTT brocker
 **************************************************************************/
void MQTTpublish() {
      // Publish and subscribe

   //if (!mqttclient.connected())  MQTTconnect();         // if not connected yet, do it
   //if (mqttstate != 0)  MQTTconnect();         // if not connected yet, do it

    MQTTconnect();
    Serial.println(F("Publishing latest datas."));
    char buffer[6];
    
    sprintf(buffer, "%.2f", tempC);                           // convert float to string before publishing
    mqttclient.publish(TOPIC_TEMP, buffer , mqtt_retained );  // publish topic to broker Last parameter "retained" is 0 or 1
    sprintf(buffer, "%.2f", humi);                            // convert float to string before publishing
    mqttclient.publish(TOPIC_HUMI, buffer, mqtt_retained  );
    sprintf(buffer, "%.1f", press);
    mqttclient.publish(TOPIC_PRESS, buffer, mqtt_retained  );
    
    #ifdef WITH_WIND
      //memset(buffer, 0, sizeof buffer);
      sprintf(buffer, "%d", windDir_avg2m);
      mqttclient.publish(TOPIC_WINDDIR, buffer,mqtt_retained);
      sprintf(buffer, "%.1f", windSpeed_avg2m);
      mqttclient.publish(TOPIC_WINDSPEED, buffer,mqtt_retained);
      sprintf(buffer, "%d", windgustDir);
      mqttclient.publish(TOPIC_GUSTDIR, buffer,mqtt_retained);
      sprintf(buffer, "%.1f", windgustSpeed);
      mqttclient.publish(TOPIC_GUSTSPEED, buffer,mqtt_retained);
      // report if the RS485 sensors are responding
      if (RS485WindSpeedSensorTimeout) mqttclient.publish(TOPIC_WINDSPEEDSENSOR, "0" ,mqtt_retained);
      else mqttclient.publish(TOPIC_WINDSPEEDSENSOR, "1" ,mqtt_retained);
      if (RS485WindDirSensorTimeout) mqttclient.publish(TOPIC_WINDDIRSENSOR, "0" ,mqtt_retained);
      else mqttclient.publish(TOPIC_WINDDIRSENSOR, "1" ,mqtt_retained);
    #endif

    #ifdef WITH_RAIN
      sprintf(buffer, "%.2f", rain1hmm);                  // convert float to string before publishing
      mqttclient.publish(TOPIC_RAIN, buffer , mqtt_retained  );
      sprintf(buffer, "%.2f", rain24hmm);               // convert float to string before publishing
      mqttclient.publish(TOPIC_RAIN24H, buffer , mqtt_retained  );
    #endif
    mqttclient.disconnect();
  }
  #endif

/**************************************************************************
 * after OTA upload
 **************************************************************************/
  void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
  //ESP.restart();
}
