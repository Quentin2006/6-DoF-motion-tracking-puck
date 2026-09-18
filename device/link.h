#pragma once
#include "packet.h"
#include "video.h"
#include "frame.h"
#include <Arduino.h>

class UsbLink {
public:
  // RX sizing note: the panel link is the 1a86:55d3 bridge (real UART), so
  // baud is wire speed and 921600 keeps a compressed 10 fps stream in budget.
  // Always returns true (nothing to fail).
  bool begin(unsigned long baud = 921600, size_t rxBuf = 4096);

  // Streaming pump: consume at most one segment per call, return at most one
  // video chunk (never a drain — that would starve 500 Hz fusion). Parses
  // the 8 B header into hdr and copies the payload into `out`, which must
  // be 4-byte aligned and hold VIDEO_PAYLOAD_MAX bytes with
  // hdr.len == VIDEO_PAYLOAD_MAX (Display::chunk blits it via SPI DMA word
  // reads). Returns payload bytes, or 0 when no complete FRAME_VIDEO frame
  // arrived / the header is short / the length is out of range.
  // Stash contract: a single Serial segment often holds several frames'
  // bytes, so input bytes after the returned chunk are kept in stash_ for
  // the next call — never discarded. Without the stash those trailing bytes
  // would be silently dropped on every multi-chunk segment, losing rows.
  int pollVideoChunk(uint8_t *out, VideoChunkHdr &hdr);

  // Build the 36 B v2 packet (stamp = micros()) and write one FRAME_POSE
  // frame to Serial. ~250 Hz: alternates send/skip per call, matching the
  // 500 Hz tick rate. Returns false when dropped by the backpressure guard.
  bool sendQuat(const float quat[4], const float gyro[3]);

  // Throughput counters (Step 2 instrumentation): cumulative since boot,
  // deltas computed by the 1 Hz reporter.
  // Per-row newest-wins note: Display::chunkInner ignores chunks whose frame
  // id is older than the row's last painted id, so stale rows never paint
  // over a newer frame. Ignored-stale is not counted; frameDrops_ stays 0
  // (fd field kept for 1 Hz report compat).
  uint32_t sent() const { return pktSeq_; }
  uint32_t dropped() const { return dropped_; }
  uint32_t rxBytes() const { return rxBytes_; }
  uint32_t chunks() const { return chunks_; }
  uint32_t decodeDrops() const { return decodeDrops_; }
  uint32_t frameDrops() const { return frameDrops_; }
  FrameParser parser_;
  uint32_t pktSeq_ = 0;
  uint32_t dropped_ = 0;
  uint32_t rxBytes_ = 0;
  uint32_t chunks_ = 0;
  uint32_t decodeDrops_ = 0;
  uint32_t frameDrops_ = 0;
  // Sized 2x one segment: pollVideoChunk feeds at most stash + one fresh
  // segment (2*kSegMax) per call, so the stashed tail after a returned chunk
  // (tail < inN <= 2*kSegMax) always fits — the overflow-drop path in
  // pollVideoChunk is defensive-only, never taken. A 1x stash overflowed on
  // every multi-frame RLE burst (small frames complete early, tail > stash),
  // discarding the oldest unconsumed bytes and desyncing the parser.
  uint8_t stash_[2 * (FRAME_MAX_PAYLOAD + 16)];
  // Valid length of stash_ in bytes.
  size_t stashN_ = 0;
  bool sendNow_ = true;
};
