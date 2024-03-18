#include <Adafruit_GFX.h>
#include <logger.h>
#include <Wire.h>
#include <vector>
#include "pins_config.h"
#include "display.h"
#include "settings.h"

extern logging::Logger logger;
extern int screenBrightness; //from 1 to 255 to regulate brightness of oled scren
extern bool RS485WindSpeedSensorTimeout; // flag to check if the wind sensor is answering
extern bool RS485WindDirSensorTimeout; // flag to check if the wind sensor is answering

#if defined(HAS_SH110X) || defined(TTGO_T_Beam_S3_SUPREME_V3)
  #undef ssd1306
#endif

// unselect SH110X if board uses a SSD1306
#ifdef HAS_SH110X
  #include <Adafruit_SH110X.h>
#else
  #include <Adafruit_SSD1306.h>
  #define ssd1306
#endif

#define SYM_HEIGHT 14
#define SYM_WIDTH  16

// custom image for broken RS485 link
static const unsigned char nolink [] PROGMEM = {
	0x40, 0x00, 0x60, 0x78, 0x30, 0xcc, 0x18, 0x06, 0x0c, 0x06, 0x06, 0x06, 0x13, 0x8c, 0x31, 0xc8, 
	0x60, 0x60, 0x60, 0x30, 0x60, 0x18, 0x33, 0x8c, 0x1e, 0x06, 0x00, 0x02
};


// T-Beams bought with soldered OLED Screen comes with only 4 pins (VCC, GND, SDA, SCL)
// If your board didn't come with 4 pins OLED Screen and comes with 5 and one of them is RST...
// Uncomment Next Line (Remember ONLY if your OLED Screen has a RST pin). This is to avoid memory issues.
//#define OLED_DISPLAY_HAS_RST_PIN

#ifdef ssd1306
  Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST);
#else
  Adafruit_SH1106G display(128, 64, &Wire, OLED_RST);
#endif

// cppcheck-suppress unusedFunction
void setup_display() {
  delay(500);
  #ifdef OLED_DISPLAY_HAS_RST_PIN // 
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(20);
    digitalWrite(OLED_RST, HIGH);
  #endif

  Wire.begin(I2C_SDA, I2C_SCL);
  #ifdef ssd1306
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) {
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "SSD1306", "allocation failed!");
    while (true) {
    }
  }
  #else
  if (!display.begin(0x3c, true)) {
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "SH1106", "allocation failed!");
    while (true) {
    }
  }
  #endif
  display.clearDisplay();
  #ifdef ssd1306
  display.setTextColor(WHITE);
  #else
  display.setTextColor(SH110X_WHITE);
  #endif
  display.setTextSize(1);
  display.setCursor(0, 0);
  #ifdef ssd1306
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(screenBrightness);
  #endif

  display.display();
}

// cppcheck-suppress unusedFunction
void display_toggle(bool toggle) {
  if (toggle) {
    #ifdef ssd1306
    display.ssd1306_command(SSD1306_DISPLAYON);
    #endif
  } else {
    #ifdef ssd1306
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    #endif
  }
}

// cppcheck-suppress unusedFunction
void show_display(String header, int wait) {
  display.clearDisplay();
  #ifdef ssd1306
  display.setTextColor(WHITE);
  #else
  display.setTextColor(SH110X_WHITE);
  #endif
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(header);
  #ifdef ssd1306
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(screenBrightness);
  #endif
  display.display();
  delay(wait);
}

// cppcheck-suppress unusedFunction
void show_display(String header, String line1, int wait) {
  display.clearDisplay();
  #ifdef ssd1306
  display.setTextColor(WHITE);
  #else
  display.setTextColor(SH110X_WHITE);
  #endif
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(header);
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.println(line1);
  #ifdef ssd1306
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(screenBrightness);
  #endif
  display.display();
  delay(wait);
}

// cppcheck-suppress unusedFunction
void show_display(String header, String line1, String line2, int wait) {
  display.clearDisplay();
  #ifdef ssd1306
  display.setTextColor(WHITE);
  #else
  display.setTextColor(SH110X_WHITE);
  #endif
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(header);
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.println(line1);
  display.setCursor(0, 26);
  display.println(line2);
  #ifdef ssd1306
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(screenBrightness);
  #endif
  display.display();
  delay(wait);
}

// cppcheck-suppress unusedFunction
void show_display(String header, String line1, String line2, String line3, int wait) {
  display.clearDisplay();
  #ifdef ssd1306
  display.setTextColor(WHITE);
  #else
  display.setTextColor(SH110X_WHITE);
  #endif
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(header);
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.println(line1);
  display.setCursor(0, 26);
  display.println(line2);
  display.setCursor(0, 36);
  display.println(line3);
  #ifdef ssd1306
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(screenBrightness);
  #endif
  display.display();
  delay(wait);
}

// cppcheck-suppress unusedFunction
void show_display(String header, String line1, String line2, String line3, String line4, int wait) {
  display.clearDisplay();
  #ifdef ssd1306
  display.setTextColor(WHITE);
  #else
  display.setTextColor(SH110X_WHITE);
  #endif
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(header);
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.println(line1);
  display.setCursor(0, 26);
  display.println(line2);
  display.setCursor(0, 36);
  display.println(line3);
  display.setCursor(0, 46);
  display.println(line4);
  #ifdef ssd1306
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(screenBrightness);
  #endif
  display.display();
  delay(wait);
}

// cppcheck-suppress unusedFunction
void show_display(String header, String line1, String line2, String line3, String line4, String line5, int wait) {
  display.clearDisplay();
  #ifdef ssd1306
  display.setTextColor(WHITE);
  #else
  display.setTextColor(SH110X_WHITE);
  #endif
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(header);
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.println(line1);
  display.setCursor(0, 26);
  display.println(line2);
  display.setCursor(0, 36);
  display.println(line3);
  display.setCursor(0, 46);
  display.println(line4);
  display.setCursor(0, 56);
  display.println(line5);
  #ifdef ssd1306
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(screenBrightness);
  #endif

    // if RS485 linkbroken, display a warning on OLED
    if (RS485WindSpeedSensorTimeout || RS485WindDirSensorTimeout) {
      #if defined(ssd1306)
        display.drawBitmap(110, 0,  nolink, 16, 14, WHITE);
      #else  
        display.drawBitmap(110, 0,  nolink, 16, 14, SH110X_WHITE);
      #endif
    }

  display.display();
  delay(wait);
}