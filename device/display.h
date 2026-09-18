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
  // Draw engine video chunks. len <= 0 (no datagram) is a no-op.
  // beginBurst opens one SPI transaction, chunkInner blits a single chunk
  // inside it, endBurst closes it. The loop() drain skips the transaction
  // entirely when no chunk arrived.
  void beginBurst();
  void chunkInner(const VideoChunkHdr &hdr, const uint16_t *px, int len);
  void endBurst();
  // Solid RGB cycle while no video has arrived recently, so it never fights
  // the video stream.
  void tick(unsigned long now);

private:
  Adafruit_ST7789 tft_;
  unsigned long lastChunkMs_ = 0;
  unsigned long lastColorMs_ = 0;
  uint8_t colorIdx_ = 0;
  // Per-row newest-wins store: full 320x240 RGB565 frame in DRAM (~150 KB
  // BSS; compile proves ~297 KB RAM headroom). Each chunk copies its 640
  // words into its 2-row slice, so every received row paints exactly once
  // and a lost chunk self-heals on the next dirty frame for that row.
  static uint16_t frameBuf_[VIDEO_W * VIDEO_H];
  // Last painted frame id per row (chunk idx), init 0. chunkInner paints a
  // row only when hdr.frame is same-or-newer (u16 wrapping compare) than
  // rowFrame_[idx]; older ids for that row are ignored, never painting stale
  // over newer. No frame-level barrier: rows advance independently, so no
  // idx0 adoption stall is possible.
  uint16_t rowFrame_[VIDEO_CHUNKS] = {};
};
