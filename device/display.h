#pragma once
#include <cstdint>
#include "config.h"
#include "video.h"
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
  // Partial chunk write: push w*h RGB565 LE pixels at (x, y). Ignored when
  // null or dims out of the 320x240 landscape-inverted video space. First
  // call latches VIDEO_ROTATION; the still-image present() path keeps
  // TFT_ROTATION.
  void writeChunk(int x, int y, int w, int h, const uint16_t *px);
  // Solid fill via fillScreen; RAM-free path for status/demo colors.
  void fill(uint16_t color);

private:
  Adafruit_ST7789 tft_;
  bool videoMode_ = false;
};
