#pragma once
#include "config.h"
#include "video.h"
#include <Adafruit_ST7789.h>
#include <SPI.h>

class Display {
public:
  Display();
  // Best-effort bring-up. Tracking streams even if the panel is unplugged.
  void begin();
  // Draw one engine video chunk. len <= 0 (no datagram) is a no-op.
  void chunk(const VideoChunkHdr &hdr, const uint16_t *px, int len);
  // Solid RGB cycle while no video has arrived recently, so it never fights
  // the video stream.
  void tick(unsigned long now);

private:
  Adafruit_ST7789 tft_;
  unsigned long lastChunkMs_ = 0;
  unsigned long lastColorMs_ = 0;
  uint8_t colorIdx_ = 0;
};
