#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

// Video wire contract: engine -> device, one stereo pair per frame, as one or
// two baseline JPEGs covering only the rectangles that changed, cut into
// fragments and sent as FRAME_VIDEO frames (device/frame.h) over the single
// USB-CDC port.
//
// Canvas: 640x240, the left eye at x in [0,320) and the right at [320,640) —
// one panel per half. Painting is per dirty rectangle and in *eye-local*
// coordinates (FRAME_VIDEO_RECT), so the rect names its panel and the device
// decodes nothing that did not change: a full 640x240 decode is ~17 ms of
// JPEGDEC on this chip (measured, pixel-bound — quality does not move it),
// which capped the panel near 31 fps however little of the frame differed.
//
// The sender paces fragments inside the frame interval, never a whole image at
// once: the USB-Serial-JTAG RX FIFO is 64 B, so a burst overflows it and
// shreds the frame (measured: 10.4 kB every 34 ms loses ~30% of its bytes, the
// same average rate spread per fragment loses none). A fragment lost that way
// costs exactly one frame — the next one starts clean.
//
// Panel mounting, including the fact that the two panels are rotated 180 deg
// from each other, is display hardware: see TFT_ROTATION in config.h.
constexpr int VIDEO_W = 640;
constexpr int VIDEO_H = 240;
constexpr int VIDEO_PANEL_W = 320; // one panel's width, i.e. one eye
constexpr size_t VIDEO_HDR_LEN = 8;
constexpr size_t VIDEO_FRAG_MAX = 1280;  // JPEG bytes per fragment; 8 + this = FRAME_MAX_PAYLOAD
constexpr size_t VIDEO_JPEG_MAX = 16384; // device assembly buffer, and the largest blob accepted

// Dirty-rectangle grid. 16x16 is exactly the 4:2:0 MCU, so a rectangle that is
// 16-px aligned on every side is a whole number of MCUs and a whole number of
// tiles, and JPEGDEC's block groups (8 MCUs x 1 at this geometry) never
// straddle a rectangle edge either. Rectangles are what make the decode cost
// track the *changed* area instead of the canvas: the SPI blit is what is left
// (2*153600 B at the measured 3.38 MB/s = 90.8 ms for a full both-panel
// repaint, which is why nothing repaints all of it unless it must).
constexpr size_t VIDEO_TILE = 16;
constexpr size_t VIDEO_RECTS_MAX = 4; // a dirty bbox and a repair quadrant per eye, at most
constexpr size_t VIDEO_RECT_LEN = 11; // eye u8 | x,y,w,h u16 LE | jpeg_len u16 LE

// One rectangle: which panel, where inside it, and how many bytes of the
// frame's blob belong to it. Eye-local coordinates, so nothing has to split
// canvas space twice and a rectangle can never span both panels. Most frames
// carry one rectangle per eye; a repair frame adds a quadrant per eye, which is
// how a whole-canvas refresh is spread over KEYFRAME_STEPS frames instead of
// costing one frame 5x its budget in SPI.
struct VideoRect {
  uint8_t eye; // 0 = left half / panel 0, 1 = right half / panel 1
  uint16_t x, y, w, h;
  uint16_t jpeg_len;
};

struct VideoRectsHdr {
  uint16_t frame;
  uint16_t rects;
  VideoRect rect[VIDEO_RECTS_MAX];
};

// Parse and validate a whole rect payload (4 + VIDEO_RECT_LEN * rects bytes):
// false if it does not describe rectangles this device can paint. Every bound
// is checked here because the decode path trusts it: a rect past the panel edge
// would blit outside the address window, and a rect that is not MCU-aligned
// would not line up with the JPEG that follows.
inline bool parse_video_rects_hdr(const uint8_t *pkt, size_t n, VideoRectsHdr &hdr) {
  if (n < 4) {
    return false;
  }
  memcpy(&hdr.frame, pkt + 0, 2);
  memcpy(&hdr.rects, pkt + 2, 2);
  const size_t count = hdr.rects;
  if (count == 0 || count > VIDEO_RECTS_MAX || n < 4 + count * VIDEO_RECT_LEN) {
    return false;
  }
  for (size_t i = 0; i < count; i++) {
    const uint8_t *r = pkt + 4 + i * VIDEO_RECT_LEN;
    VideoRect &out = hdr.rect[i];
    out.eye = r[0];
    memcpy(&out.x, r + 1, 2);
    memcpy(&out.y, r + 3, 2);
    memcpy(&out.w, r + 5, 2);
    memcpy(&out.h, r + 7, 2);
    memcpy(&out.jpeg_len, r + 9, 2);
    if (out.eye > 1 || out.w == 0 || out.h == 0 || out.jpeg_len == 0) {
      return false;
    }
    if ((out.x % VIDEO_TILE) != 0 || (out.y % VIDEO_TILE) != 0 || (out.w % VIDEO_TILE) != 0 ||
        (out.h % VIDEO_TILE) != 0) {
      return false;
    }
    if (out.x + out.w > VIDEO_PANEL_W || out.y + out.h > VIDEO_H) {
      return false;
    }
  }
  // A frame is one JPEG per rectangle, concatenated in the blob the fragments
  // carry; the caller checks the assembled length against this sum.
  size_t total = 0;
  for (size_t i = 0; i < count; i++) {
    total += hdr.rect[i].jpeg_len;
  }
  return total <= VIDEO_JPEG_MAX;
}

// Total JPEG bytes a parsed rect set expects in the blob.
inline size_t video_rects_blob_len(const VideoRectsHdr &hdr) {
  size_t total = 0;
  for (size_t i = 0; i < hdr.rects && i < VIDEO_RECTS_MAX; i++) {
    total += hdr.rect[i].jpeg_len;
  }
  return total;
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
