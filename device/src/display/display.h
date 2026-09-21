#pragma once
#include "../../config.h"
#include <Adafruit_ST7789.h>
#include <SPI.h>

class Display {
public:
  Display();
  // Best-effort bring-up. Tracking streams even if the panel is unplugged.
  void begin();
  // Draw engine video. beginBurst opens one SPI transaction, blitBlock paints
  // one decoded JPEG block (a JPEGDEC MCU group) inside it, endBurst closes it.
  // The video task skips the transaction entirely when no frame was painted.
  void beginBurst();
  void blitBlock(int x, int y, int w, int h, const uint16_t *px);
  void endBurst();
  // A video frame arrived (whether or not it needed repainting), so the idle
  // RGB cycle stays away.
  void markVideo(unsigned long now) { lastVideoMs_ = now; }
  // Solid RGB cycle while no video has arrived recently, so it never fights
  // the video stream. Returns true when it repainted, i.e. the caller's idea
  // of what is on the panel is stale.
  bool tick(unsigned long now);

private:
  Adafruit_ST7789 tft_;
  unsigned long lastVideoMs_ = 0;
  unsigned long lastColorMs_ = 0;
  uint8_t colorIdx_ = 0;
};
