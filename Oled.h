#ifndef OLED_H
#define OLED_H

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

class Oled {
  public:
    Oled() {}
    ~Oled() {}
    void init() {
      Wire.begin(22, 21);
      // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
      if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        for (;;); // Don't proceed, loop forever
      }
      display.display();
      // Clear the buffer
      display.clearDisplay();
    }

    void drawString(String str) {
      display.clearDisplay();

      display.setTextSize(2);      // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE); // Draw white text
      display.setCursor(0, 0);     // Start at top-left corner
      display.cp437(true);         // Use full 256 char 'Code Page 437' font

      for (int i = 0, l = str.length(); i < l; i++) {
        display.write(str[i]);
      }

      display.display();
    }

    void drawString(String str, uint8_t textSize) {
      display.clearDisplay();

      display.setTextSize(textSize);      // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE); // Draw white text
      display.setCursor(0, 0);     // Start at top-left corner
      display.cp437(true);         // Use full 256 char 'Code Page 437' font

      for (int i = 0, l = str.length(); i < l; i++) {
        display.write(str[i]);
      }

      display.display();
    }
};

#endif
