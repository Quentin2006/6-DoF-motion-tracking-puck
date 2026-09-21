#include <Arduino.h>
#include "display.h"
#include "video.h" // VIDEO_ROTATION

Display::Display() : tft_(TFT_CS, TFT_DC, TFT_RST) {}

void Display::begin() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  tft_.init(TFT_W, TFT_H);
  tft_.setSPISpeed(TFT_SPI_FREQ);
  // Panel is mounted landscape, matching the engine's video frame layout.
  tft_.setRotation(VIDEO_ROTATION);
  tft_.fillScreen(ST77XX_BLACK);
  lastColorMs_ = millis();
}

void Display::beginBurst() { tft_.startWrite(); }

void Display::endBurst() { tft_.endWrite(); }

void Display::blitBlock(int x, int y, int w, int h, const uint16_t *px) {
  tft_.setAddrWindow((uint16_t)x, (uint16_t)y, (uint16_t)w, (uint16_t)h);
  // Adafruit's 2nd argument is a PIXEL count (w*h*2 bytes on the wire).
  // bigEndian=false is the word swap the panel needs: SPIClass::writePixels
  // swaps in hardware on ESP32.
  tft_.writePixels(const_cast<uint16_t *>(px), (uint32_t)(w * h), true, false);
  lastVideoMs_ = millis();
}

bool Display::tick(unsigned long now) {
  const bool videoIdle = lastVideoMs_ == 0 || (unsigned long)(now - lastVideoMs_) > VIDEO_IDLE_MS;
  const bool due = (unsigned long)(now - lastColorMs_) >= DISPLAY_COLOR_PERIOD_MS;
  if (!videoIdle || !due) {
    return false;
  }
  lastColorMs_ = now;
  static const uint16_t kColors[3] = {ST77XX_RED, ST77XX_GREEN, ST77XX_BLUE};
  colorIdx_ = (uint8_t)((colorIdx_ + 1) % 3);
  tft_.fillScreen(kColors[colorIdx_]);
  return true;
}
