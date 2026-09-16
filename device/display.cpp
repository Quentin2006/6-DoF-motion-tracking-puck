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

void Display::fill(uint16_t color) {
  tft_.fillScreen(color);
}
