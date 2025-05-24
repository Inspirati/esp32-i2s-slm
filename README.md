# Branch esp32-i2s-slm_pio-vscode featuring support for platformio and vscode.

<a href="https://github.com/Inspirati/esp32-i2s-slm/tree/esp32-i2s-slm_pio-vscode">esp32-i2s-slm with PlatformIO & vscode support</a>

This new branch also features limited console support and various alternative options for the IIR filter algorithm.

Developed and tested on an ESP32 D1 mini with 64x48 OLED shield using an INMP441 microphone and build chain (et-al):

PLATFORM: Espressif 32 (51.3.5) > Espressif ESP32 Dev Module
HARDWARE: ESP32 240MHz, 320KB RAM, 4MB Flash
PACKAGES: 
 - framework-arduinoespressif32 @ 3.0.5 
 - framework-arduinoespressif32-libs @ 5.1.0+sha.33fbade6b8 
 - tool-esptoolpy @ 4.7.5 
 - toolchain-xtensa-esp32 @ 12.2.0+20230208

Including dependency:
 - ESP8266 and ESP32 OLED driver for SSD1306 displays @ 4.6.1+sha.f90368e

--------------------------------------------------------------------------------
The following content is preservered from the <a href="https://github.com/ikostoski/esp32-i2s-slm">original</a> author, Ivan Kostoski
--------------------------------------------------------------------------------

# Sound Level Meter with Arduino IDE, ESP32 and I<sup>2</sup>S MEMS microphone

Arduino/ESP32 Sound Level Meter (SLM) using inexpensive, factory calibrated, digital I2S MEMS microphone and digital IIR filters (vs. FFT) for equalization and A-weighting.

The basic idea is:

![Basic principle](./misc/esp32-i2s-slm-bp.svg)

And the microphone response after equalization should look like:

![Adjusted frequency response](./misc/ics-43434-afr.svg)

Theoretically, i.e. with factory calibrated ICS-4343x, this should get you ±1dB(A) measurement within 20Hz-20KHz range.

The code in this repository is mostly intended as example how you can integrate resonable noise measurement (i.e. *L*<sub>Aeq</sub>, Equivalent Continuous Sound Level) in your projects.

You can find a bit more information in my [hackday.io](https://hackaday.io/project/166867-esp32-i2s-slm) project.
