#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

// Video wire contract: engine -> device raw 320x240 RGB565 frame fragments on
// UDP VIDEO_UDP_PORT, landscape (90 deg CW from portrait 0).
constexpr uint16_t VIDEO_UDP_PORT = 3334;
constexpr int VIDEO_W = 320;
constexpr int VIDEO_H = 240;
constexpr size_t VIDEO_PAYLOAD_MAX = 1280;                          // 2 rows x 320 px x 2 B
constexpr int VIDEO_CHUNK_ROWS = VIDEO_PAYLOAD_MAX / (VIDEO_W * 2); // 2 rows per datagram
constexpr uint16_t VIDEO_CHUNKS = VIDEO_H / VIDEO_CHUNK_ROWS;       // 120 datagrams = one frame
constexpr uint8_t VIDEO_ROTATION = 1;                               // landscape logical space
constexpr size_t VIDEO_HDR_LEN = 8;

// Header (8 B, all u16 LE): frame_id | chunk_idx | chunk_count | payload_len.
// frame_id wraps; the device writes whatever arrives, so a torn frame
// self-heals on the next frame.
struct VideoChunkHdr {
  uint16_t frame;
  uint16_t idx;
  uint16_t count;
  uint16_t len;
};

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
