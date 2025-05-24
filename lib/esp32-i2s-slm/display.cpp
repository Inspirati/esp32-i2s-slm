#include <Arduino.h>
#include <driver/i2s.h>
#include "display.h"

//#define USE_OLED_DISPLAY
#ifndef USE_OLED_DISPLAY
void display_init() {}
void display_loop() {}
void display_update(double value, double level, int indicator) {}
#else

#define LEQ_UNITS  "LAeq"      // customize based on weighting used
#define DB_UNITS   "dBA"       // customize based on weighting used

//
// Setup your display library (and geometry) here
//
// ThingPulse/esp8266-oled-ssd1306, you may need the latest source and PR#198 for 64x48
#include <SSD1306Wire.h>
#define OLED_GEOMETRY     GEOMETRY_64_48
//#define OLED_GEOMETRY GEOMETRY_128_32
//#define OLED_GEOMETRY GEOMETRY_128_64
#define OLED_FLIP_V       1
//SSD1306Wire display(0x3c, SDA, SCL, OLED_GEOMETRY);
SSD1306Wire display(0x3d, SDA, SCL, OLED_GEOMETRY);

void display_init() {
    display.init();
#if (OLED_FLIP_V > 0)
    display.flipScreenVertically();
#endif
    display.setFont(ArialMT_Plain_16);
}

void display_loop() {
}

void display_update(double value, double level, int indicator) {
    //
    // Example code that displays the measured value.
    // You should customize the below code for your display
    // and display library used.
    //

    display.clear();

    // It is important to somehow notify when the device is out of its range
    // as the calculated values are very likely with big error
    if (indicator > 1) {
        // Display 'Overload' if dB value is over the AOP
        display.drawString(0, 24, "Overload");
    } else if (indicator > 0) {
        // Display 'Low' if dB value is below noise floor
        display.drawString(0, 24, "Low");
    }
    // The 'short' Leq line
    uint16_t len = min(max(0, int(level * (display.getWidth()-1))), display.getWidth()-1);
    display.drawHorizontalLine(0, 0, len);
    display.drawHorizontalLine(0, 1, len);
    display.drawHorizontalLine(0, 2, len);

    // The Leq numeric decibels
    display.drawString(0, 4, String(value, 1) + " " + DB_UNITS);

    display.display();
}

#endif // USE_OLED_DISPLAY

