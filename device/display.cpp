#include <cstring>
#include <Arduino.h>
#include "display.h"

// Static DRAM store: 320*240 u16 = 153600 B BSS (see display.h).
uint16_t Display::frameBuf_[VIDEO_W * VIDEO_H];

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

void Display::chunkInner(const VideoChunkHdr &hdr, const uint16_t *px, int len) {
  if (len != (int)VIDEO_PAYLOAD_MAX || hdr.idx >= VIDEO_CHUNKS) {
    return;
  }
  // Same-or-newer (u16 wrap: distance < 32768 is newer, 0 is same) paints;
  // older ids for this row are stale and ignored, never overpainting newer.
  if (hdr.frame != rowFrame_[hdr.idx] &&
      (uint16_t)(hdr.frame - rowFrame_[hdr.idx]) >= 0x8000u) {
    return;
  }
  uint16_t *dst = &frameBuf_[(size_t)hdr.idx * VIDEO_WORDS_PER_CHUNK];
  memcpy(dst, px, VIDEO_PAYLOAD_MAX);
  tft_.setAddrWindow(0, (uint16_t)(hdr.idx * VIDEO_CHUNK_ROWS), VIDEO_W, VIDEO_CHUNK_ROWS);
  // Adafruit's 2nd argument is a PIXEL count (len * 2 bytes on the wire).
  tft_.writePixels(dst, VIDEO_W * VIDEO_CHUNK_ROWS, true, false);
  rowFrame_[hdr.idx] = hdr.frame;
  lastChunkMs_ = millis();
}

void Display::tick(unsigned long now) {
  const bool videoIdle = lastChunkMs_ == 0 || (unsigned long)(now - lastChunkMs_) > 10000UL;
  if (!videoIdle || (unsigned long)(now - lastColorMs_) < DISPLAY_COLOR_PERIOD_MS) {
    return;
  }
  lastColorMs_ = now;
  static const uint16_t kColors[3] = {ST77XX_RED, ST77XX_GREEN, ST77XX_BLUE};
  colorIdx_ = (uint8_t)((colorIdx_ + 1) % 3);
  tft_.fillScreen(kColors[colorIdx_]);
}
