#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

// Video wire contract: engine -> device 640x240 baseline JPEG (the stereo
// pair: left eye | right eye, side by side), cut into fragments and sent as
// FRAME_VIDEO frames (device/frame.h) over the single USB-CDC port, landscape
// (90 deg CW from portrait 0).
//
// The sender paces one fragment per slice of the frame interval, never a whole
// image at once: the USB-Serial-JTAG RX FIFO is 64 B, so a burst overflows it
// and shreds the frame (measured: 10.4 kB every 34 ms loses ~30% of its bytes,
// the same average rate spread per fragment loses none). A fragment lost that
// way costs exactly one frame — the next one starts clean.
//
// The canvas is split in half geometrically, not by an eye field in the
// header: tile columns 0..19 are the left panel, 20..39 the right one. Which
// panel is the physical left eye is a wiring fact (see panel_tile_x).
constexpr int VIDEO_W = 640;
constexpr int VIDEO_H = 240;
constexpr int VIDEO_PANEL_W = 320; // one panel's width in canvas space
constexpr uint8_t VIDEO_ROTATION = 1; // landscape logical space
constexpr size_t VIDEO_HDR_LEN = 8;
constexpr size_t VIDEO_FRAG_MAX = 1280;  // JPEG bytes per fragment; 8 + this = FRAME_MAX_PAYLOAD
constexpr size_t VIDEO_JPEG_MAX = 49152; // device assembly buffer, and the largest frame accepted

// Dirty-tile grid. 16x16 is exactly the 4:2:0 MCU, so JPEGDEC's block groups
// (8 MCUs x 1 at this geometry) tile-align: a block is a whole number of
// tiles, and a tile never straddles two blocks. A tile is repainted only when
// the host says it changed (FRAME_VIDEO_MASK), because a full both-panel blit
// costs 2*153600 B at the measured 3.38 MB/s = 90.8 ms — 9 fps on its own.
constexpr size_t VIDEO_TILE = 16;
constexpr size_t VIDEO_TILES = (VIDEO_W / VIDEO_TILE) * (VIDEO_H / VIDEO_TILE); // 600
constexpr size_t VIDEO_MASK_BYTES = (VIDEO_TILES + 7) / 8;                      // 75

// Panel index and in-panel x for a canvas tile column, or -1 if out of range.
// Columns 0..19 are panel 0, 20..39 panel 1 — both VIDEO_PANEL_W/VIDEO_TILE
// columns wide. The in-panel x is in the panel's own 320x240 (rotated) space.
inline int panel_tile_x(int tile_x, int *local_x) {
  constexpr int cols = VIDEO_W / VIDEO_TILE;
  constexpr int per_panel = VIDEO_PANEL_W / VIDEO_TILE;
  if (tile_x < 0 || tile_x >= cols) {
    return -1;
  }
  *local_x = (tile_x % per_panel) * (int)VIDEO_TILE;
  return tile_x / per_panel;
}

// Header (8 B, all u16 LE): frame_id | frag_idx | frag_count | frag_len.
// frame_id wraps. Fragments must arrive in order: the byte stream is ordered,
// so any other index means bytes were lost and the frame cannot be repaired.
struct VideoFragHdr {
  uint16_t frame;
  uint16_t frag;
  uint16_t count;
  uint16_t len;
};

// Parse header; false if buf smaller than VIDEO_HDR_LEN.
inline bool parse_video_frag_hdr(const uint8_t *pkt, size_t n, VideoFragHdr &hdr) {
  if (n < VIDEO_HDR_LEN) {
    return false;
  }
  memcpy(&hdr.frame, pkt + 0, 2);
  memcpy(&hdr.frag, pkt + 2, 2);
  memcpy(&hdr.count, pkt + 4, 2);
  memcpy(&hdr.len, pkt + 6, 2);
  return true;
}

// ---------------------------------------------------------------- dirty mask
// FRAME_VIDEO_MASK payload (4 + VIDEO_MASK_BYTES = 79 B, all u16 LE):
// frame_id | tiles | mask.
//
// The device cannot work dirtiness out for itself: a per-eye 320x240 RGB565
// shadow is 307 KB and it has 82 KB free. The host already holds the previous
// RGBA frame, so it computes the mask and sends it as 75 bytes per frame.
// Bit t covers tile t (row-major over VIDEO_W/VIDEO_TILE columns, VIDEO_TILE^2
// px); only tiles == VIDEO_TILES is accepted, and the mask must ride the frame
// whose id it names, or the device paints blind.
struct VideoMaskHdr {
  uint16_t frame;
  uint16_t tiles;
};

// Parse and validate a whole mask payload; false if it is not a full mask for
// this canvas. Accepts nothing partial: a short payload would leave the tail
// of the mask undefined and silently under-paint.
inline bool parse_video_mask_hdr(const uint8_t *pkt, size_t n, VideoMaskHdr &hdr) {
  constexpr size_t kLen = 4 + VIDEO_MASK_BYTES;
  if (n < kLen) {
    return false;
  }
  memcpy(&hdr.frame, pkt + 0, 2);
  memcpy(&hdr.tiles, pkt + 2, 2);
  return hdr.tiles == VIDEO_TILES;
}

// Reassemble one JPEG from its fragments (must mirror send_jpeg in
// engine/src/tracking.rs, which is the producer). Loss poisons a frame: the
// assembler abandons the rest of it instead of splicing two images together,
// so the cost of a lost fragment is exactly one frame.
class VideoFragAssembler {
public:
  // true once the frame is complete; buf()/len() then hold it until the next
  // push, so the caller must consume it first.
  bool push(const VideoFragHdr &hdr, const uint8_t *data, size_t n);
  const uint8_t *buf() const { return buf_; }
  size_t len() const { return len_; }
  uint32_t dropped() const { return dropped_; }

private:
  // got_ == 0 means "expect fragment 0"; anything else is mid-frame.
  void abort() {
    got_ = 0;
    want_ = 0;
    len_ = 0;
  }
  // 16-byte aligned: this is the buffer JPEGDEC reads the JPEG out of, and its
  // S3 SIMD paths want aligned operands.
  alignas(16) uint8_t buf_[VIDEO_JPEG_MAX];
  uint16_t frame_ = 0, want_ = 0, got_ = 0;
  size_t len_ = 0;
  uint32_t dropped_ = 0;
};

inline bool VideoFragAssembler::push(const VideoFragHdr &hdr, const uint8_t *data, size_t n) {
  // Structurally impossible fragment: nonsense header, or fewer bytes than it
  // claims to carry.
  if (hdr.count == 0 || hdr.len == 0 || hdr.len > VIDEO_FRAG_MAX || hdr.frag >= hdr.count ||
      n < hdr.len) {
    dropped_++;
    return false;
  }
  // In-order only. A gap means the in-flight frame is already unrecoverable.
  if (hdr.frag != got_) {
    abort();
    dropped_++;
    return false;
  }
  if (got_ == 0) {
    frame_ = hdr.frame;
    want_ = hdr.count;
    len_ = 0;
  } else if (hdr.frame != frame_) {
    abort();
    dropped_++;
    return false;
  }
  if (len_ + hdr.len > VIDEO_JPEG_MAX) {
    abort();
    dropped_++;
    return false;
  }
  memcpy(buf_ + len_, data, hdr.len);
  len_ += hdr.len;
  got_++;
  if (got_ != want_) {
    return false;
  }
  // Complete. got_ resets so the next frame's fragment 0 is expected; len_
  // stays valid for the caller's buf()/len() read.
  got_ = 0;
  want_ = 0;
  return true;
}
