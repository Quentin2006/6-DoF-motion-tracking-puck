#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

// Video wire contract: engine -> device raw 320x240 RGB565 frame fragments.
// Landscape (90 deg CW from portrait 0).
// Quat stream (packet.h, UDP 3333) untouched; this rides a dedicated port.
constexpr uint16_t VIDEO_UDP_PORT = 3334;
constexpr int VIDEO_W = 320;
constexpr int VIDEO_H = 240;
constexpr size_t VIDEO_FRAME_BYTES = 153600; // 320*240*2
constexpr size_t VIDEO_PAYLOAD_MAX = 1280;   // 2 rows x 320 px x 2 B
constexpr uint16_t VIDEO_CHUNKS = 120;       // 120 x 1280 (all equal)
constexpr size_t VIDEO_TAIL_BYTES = 1280;    // 2 rows x 320 px x 2 B
constexpr uint8_t VIDEO_ROTATION = 1;        // landscape logical space
constexpr size_t VIDEO_HDR_LEN = 8;

// Header (8 B, all u16 LE): frame_id | chunk_idx | chunk_count | payload_len.
// frame_id wraps; stale-frame gate on device makes wrap safe.
struct VideoChunkHdr {
  uint16_t frame;
  uint16_t idx;
  uint16_t count;
  uint16_t len;
};

// Serialize header into pkt (must be >= VIDEO_HDR_LEN bytes).
inline void build_video_header(uint8_t *pkt, uint16_t frame, uint16_t idx,
                               uint16_t count, uint16_t len) {
  memcpy(pkt + 0, &frame, 2);
  memcpy(pkt + 2, &idx, 2);
  memcpy(pkt + 4, &count, 2);
  memcpy(pkt + 6, &len, 2);
}

// Parse header; false if buf smaller than VIDEO_HDR_LEN.
inline bool parse_video_header(const uint8_t *pkt, size_t n, VideoChunkHdr &hdr) {
  if (n < VIDEO_HDR_LEN) {
    return false;
  }
  memcpy(&hdr.frame, pkt + 0, 2);
  memcpy(&hdr.idx, pkt + 2, 2);
  memcpy(&hdr.count, pkt + 4, 2);
  memcpy(&hdr.len, pkt + 6, 2);
  return true;
}
