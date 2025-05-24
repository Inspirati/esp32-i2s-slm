#include <Arduino.h>
#include "esp32-i2s-slm.h"
#include "Console.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

void setup() {
  // If desired, you can lower the CPU frquency,
  // i.e. if you want to (slightly) reduce ESP32 power consumption
  //setCpuFrequencyMhz(80); // It should run as low as 80MHz
  pinMode(LED_BUILTIN, OUTPUT);
  console_init();
  slm_init();
}

void loop() {
  static int ledState = LOW;
  double leq;

  console_loop();
  if (slm_loop(leq)) {
    if (leq) {
      digitalWrite(LED_BUILTIN, ledState = !ledState);
      if (verbose) {
        Serial.printf("leq: %.1f\n", leq);
      }
    }
  }
}

