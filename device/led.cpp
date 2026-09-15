#include <Arduino.h>  // defines RGB_BUILTIN (pins_arduino.h) before config.h parses
#include "led.h"

void StatusLed::red() {
  rgbLedWrite(RGB_LED_PIN, 255, 0, 0);
}

void StatusLed::green() {
  rgbLedWrite(RGB_LED_PIN, 0, 255, 0);
}
