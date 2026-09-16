#pragma once
#include "config.h"

class StatusLed {
public:
  // Calibration/debug indicator on the onboard WS2812 (RGB_LED_PIN).
  void red();    // calibrating / booting / not ready
  void green();  // calibrated, streaming
};
