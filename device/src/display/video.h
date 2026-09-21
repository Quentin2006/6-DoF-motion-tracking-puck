#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

// Video wire contract: engine -> device 320x240 baseline JPEG, cut into
// fragments and sent as FRAME_VIDEO frames (device/frame.h) over the single
// USB-CDC port, landscape (90 deg CW from portrait 0).
//
// The sender paces one fragment per slice of the frame interval, never a whole
// image at once: the USB-Serial-JTAG RX FIFO is 64 B, so a burst overflows it
// and shreds the frame (measured: 10.4 kB every 34 ms loses ~30% of its bytes,
// the same average rate spread per fragment loses none). A fragment lost that
// way costs exactly one frame — the next one starts clean.
constexpr int VIDEO_W = 320;
constexpr int VIDEO_H = 240;
constexpr uint8_t VIDEO_ROTATION = 1; // landscape logical space
constexpr size_t VIDEO_HDR_LEN = 8;
constexpr size_t VIDEO_FRAG_MAX = 1280;  // JPEG bytes per fragment; 8 + this = FRAME_MAX_PAYLOAD
constexpr size_t VIDEO_JPEG_MAX = 49152; // device assembly buffer, and the largest frame accepted

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
