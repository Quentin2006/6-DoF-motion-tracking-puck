#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

// Video wire contract: engine -> device 320x240 RGB565 frame fragments as
// FRAME_VIDEO / FRAME_VIDEO_RLE frames (device/frame.h) over the single
// USB-CDC port, landscape (90 deg CW from portrait 0).
// Burst norm: the engine bursts only dirty 2-row chunks with no pacing (plus
// a chunk-0 heartbeat on fully static frames); the device consumes them in a
// core-0 task (src/display/video_task.cpp), so motion frames degrade via
// queue drops, never fusion stalls. Unchanged rows keep prior pixels; a lost
// chunk self-heals on the next dirty frame.
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

// Per-chunk RLE over the 640 u16 LE words of one chunk (must mirror
// rle_encode_565 in engine/src/tracking.rs). Control byte per run: bit7 = 1
// run (bits6..0 count, one following 2-byte LE value), bit7 = 0 literal
// (bits6..0 count, count following 2-byte LE words); a zero count encodes
// 128 words, so counts span 1..128. Returns false on any overrun/truncation
// or when the output is not exactly 640 words; the caller drops the chunk
// exactly like a CRC reject.
constexpr size_t VIDEO_WORDS_PER_CHUNK = VIDEO_PAYLOAD_MAX / 2; // 640
inline bool rle_decode_565(const uint8_t *in, size_t inLen, uint16_t *out640) {
  size_t ip = 0, op = 0;
  while (ip < inLen) {
    const uint8_t ctrl = in[ip++];
    const size_t run = ((size_t)(ctrl & 0x7F) == 0) ? 128 : (size_t)(ctrl & 0x7F);
    if (ctrl & 0x80) {
      if (ip + 2 > inLen || op + run > VIDEO_WORDS_PER_CHUNK) {
        return false;
      }
      uint16_t v;
      memcpy(&v, in + ip, 2);
      ip += 2;
      for (size_t k = 0; k < run; k++) {
        out640[op++] = v;
      }
    } else {
      if (ip + run * 2 > inLen || op + run > VIDEO_WORDS_PER_CHUNK) {
        return false;
      }
      for (size_t k = 0; k < run; k++) {
        uint16_t v;
        memcpy(&v, in + ip, 2);
        ip += 2;
        out640[op++] = v;
      }
    }
  }
  return op == VIDEO_WORDS_PER_CHUNK;
}
