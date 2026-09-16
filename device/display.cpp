#include <Arduino.h>
#include "display.h"

Display::Display() : tft_(TFT_CS, TFT_DC, TFT_RST) {}

bool Display::begin() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  tft_.init(TFT_W, TFT_H);
  tft_.setSPISpeed(TFT_SPI_FREQ);
  tft_.setRotation(TFT_ROTATION);
  tft_.fillScreen(ST77XX_BLACK);
  return true;
}

void Display::present(const uint16_t *frame) {
  if (!frame) {
    return;
  }
  tft_.drawRGBBitmap(0, 0, frame, TFT_W, TFT_H);
}

void Display::writeChunk(int x, int y, int w, int h, const uint16_t *px) {
  if (!px || w <= 0 || h <= 0 || x < 0 || y < 0) {
    return;
  }
  if (x + w > VIDEO_W || y + h > VIDEO_H) {
    return;
  }
  if (!videoMode_) {
    videoMode_ = true;
    tft_.setRotation(VIDEO_ROTATION);
  }
  tft_.startWrite();
  tft_.setAddrWindow((uint16_t)x, (uint16_t)y, (uint16_t)w, (uint16_t)h);
  // writePixels takes non-const; payload buffer is caller-owned scratch.
  tft_.writePixels(const_cast<uint16_t *>(px), (uint32_t)w * (uint32_t)h, true, false);
  tft_.endWrite();
}

void Display::fill(uint16_t color) {
  tft_.fillScreen(color);
}
