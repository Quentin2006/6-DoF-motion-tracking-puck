#pragma once
#include <cstdint>
#include "config.h"
#include <Adafruit_ST7789.h>
#include <SPI.h>

class Display {
public:
  Display();
  // Best-effort bring-up; returns true unconditionally (Adafruit init
  // returns void). Tracking streams even if the panel is unplugged.
  bool begin();
  // Push caller-owned frame[0 .. TFT_W*TFT_H-1], row-major RGB565
  // native-endian. Null frame: silent no-op.
  void present(const uint16_t *frame);
  // Solid fill via fillScreen; RAM-free path for status/demo colors.
  void fill(uint16_t color);

private:
  Adafruit_ST7789 tft_;
};
