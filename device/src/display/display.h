#pragma once
#include "../../config.h"
#include <Adafruit_ST7789.h>
#include <SPI.h>

// Two ST7789 panels on one SPI bus, one chip select each (TFT_CS / TFT_CS2).
// MOSI, SCLK, DC, RST and the backlight are shared, so the chip select is the
// only thing that decides which eye receives what: the panels are not
// addressable, they are two independent shift-in targets on the same wires.
class Display {
public:
  Display();
  // Best-effort bring-up. Tracking streams even if a panel is unplugged.
  void begin();
  // Draw engine video. beginBurst opens one SPI transaction that spans the
  // whole frame's rectangles, blitTile paints one canvas tile inside it, and
  // endBurst closes it — the transaction is opened before any tile is known
  // to need painting, so an all-decode-failed frame still costs one open/close.
  void beginBurst();
  // Paint one VIDEO_TILE x VIDEO_TILE tile at (x, y) inside `panel`, `px`
  // holding VIDEO_TILE contiguous RGB565 pixels per row. Coordinates are the
  // panel's own (the rect that carried the tile named the panel), and the
  // caller has already validated that the tile is inside it.
  void blitTile(int panel, int x, int y, const uint16_t *px);
  void endBurst();
  // A video frame arrived (whether or not anything needed repainting), so the
  // idle RGB cycle stays away.
  void markVideo(unsigned long now) { lastVideoMs_ = now; }
  // Solid RGB cycle while no video has arrived recently, so it never fights
  // the video stream. Returns true when it repainted, i.e. the caller's idea
  // of what is on the panels is stale.
  bool tick(unsigned long now);

private:
  // Same controller, same bus, different chip select — two instances is the
  // whole difference. Index 0 is TFT_CS, index 1 TFT_CS2 (kCsPins).
  Adafruit_ST7789 tft_[2];
  unsigned long lastVideoMs_ = 0;
  unsigned long lastColorMs_ = 0;
  uint8_t colorIdx_ = 0;
};
