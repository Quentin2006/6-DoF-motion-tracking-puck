#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

// Framed byte-stream contract over the single USB-CDC port (Serial).
// Logs (Serial.println/printf) share the port unframed; the parser skips any
// bytes that do not form a valid frame, so logs and frames coexist.
//
// Frame: magic 0x55 0xAA | type u8 | len u16 LE | payload[len] | crc u16 LE.
// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type + len bytes +
// payload only (magic and crc excluded).
// Pose payload (FRAME_POSE, len == 36) is the exact 36 bytes from
// build_quat_packet (device/packet.h); video payload (FRAME_VIDEO) is the
// existing 8-byte video header (device/src/display/video.h) + chunk bytes.
constexpr uint8_t FRAME_MAGIC_0 = 0x55;
constexpr uint8_t FRAME_MAGIC_1 = 0xAA;
constexpr uint8_t FRAME_POSE = 0x01;
constexpr uint8_t FRAME_VIDEO = 0x02;
// Same 8-byte video header + chunk bytes as FRAME_VIDEO, but the chunk bytes
// are RLE-compressed (see rle_decode_565 in device/src/display/video.h); hdr.len is the
constexpr uint8_t FRAME_VIDEO_RLE = 0x03;
constexpr size_t FRAME_MAX_PAYLOAD = 1288; // largest video payload (8 + 1280)
constexpr size_t FRAME_HDR_LEN = 5;        // magic[2] + type + len[2]
constexpr size_t FRAME_CRC_LEN = 2;

inline uint16_t frame_crc_feed(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)b << 8;
  for (int k = 0; k < 8; k++) {
    crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// CRC-16/CCITT-FALSE over `data[0..n)`.
inline uint16_t frame_crc16(const uint8_t *data, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc = frame_crc_feed(crc, data[i]);
  }
  return crc;
}

// Build one frame into `out` (must hold 7 + len bytes). Returns total bytes.
inline size_t build_frame(uint8_t *out, uint8_t type, const uint8_t *payload, uint16_t len) {
  out[0] = FRAME_MAGIC_0;
  out[1] = FRAME_MAGIC_1;
  out[2] = type;
  out[3] = (uint8_t)(len & 0xFF);
  out[4] = (uint8_t)((len >> 8) & 0xFF);
  if (len > 0) {
    memcpy(out + FRAME_HDR_LEN, payload, len);
  }
  uint16_t crc = 0xFFFF;
  crc = frame_crc_feed(crc, type);
  crc = frame_crc_feed(crc, out[3]);
  crc = frame_crc_feed(crc, out[4]);
  for (uint16_t i = 0; i < len; i++) {
    crc = frame_crc_feed(crc, payload[i]);
  }
  out[FRAME_HDR_LEN + len] = (uint8_t)(crc & 0xFF);
  out[FRAME_HDR_LEN + len + 1] = (uint8_t)((crc >> 8) & 0xFF);
  return FRAME_HDR_LEN + len + FRAME_CRC_LEN;
}

// Byte-at-a-time parser. Returns true exactly once per complete valid frame;
// `payload_out` (must hold FRAME_MAX_PAYLOAD bytes) receives the payload.
struct FrameParser {
  bool push(uint8_t byte, uint8_t &type_out, uint8_t *payload_out, uint16_t &len_out) {
    switch (state_) {
      case State::kHunt:
        if (byte == FRAME_MAGIC_0) {
          state_ = State::kGotMagic0;
        }
        return false;
      case State::kGotMagic0:
        if (byte == FRAME_MAGIC_1) {
          state_ = State::kType;
        } else if (byte == FRAME_MAGIC_0) {
          state_ = State::kGotMagic0; // overlap: 0x55 0x55 0xAA
        } else {
          state_ = State::kHunt;
        }
        return false;
      case State::kType:
        type_ = byte;
        crc_ = frame_crc_feed(0xFFFF, byte);
        state_ = State::kLenLo;
        return false;
      case State::kLenLo:
        len_ = byte;
        crc_ = frame_crc_feed(crc_, byte);
        state_ = State::kLenHi;
        return false;
      case State::kLenHi:
        len_ |= (uint16_t)((uint16_t)byte << 8);
        crc_ = frame_crc_feed(crc_, byte);
        if (len_ > FRAME_MAX_PAYLOAD) {
          reset();
          return false;
        }
        pos_ = 0;
        state_ = (len_ == 0) ? State::kCrcLo : State::kPayload;
        return false;
      case State::kPayload:
        buf_[pos_++] = byte;
        crc_ = frame_crc_feed(crc_, byte);
        if (pos_ >= len_) {
          state_ = State::kCrcLo;
        }
        return false;
      case State::kCrcLo:
        crcRx_ = byte;
        state_ = State::kCrcHi;
        return false;
      case State::kCrcHi: {
        crcRx_ |= (uint16_t)((uint16_t)byte << 8);
        bool ok = (crcRx_ == crc_);
        uint8_t t = type_;
        uint16_t l = len_;
        if (ok && l > 0) {
          memcpy(payload_out, buf_, l);
        }
        reset();
        if (ok) {
          type_out = t;
          len_out = l;
          return true;
        }
        return false;
      }
    }
    reset();
    return false;
  }

 private:
  enum class State : uint8_t {
    kHunt,
    kGotMagic0,
    kType,
    kLenLo,
    kLenHi,
    kPayload,
    kCrcLo,
    kCrcHi,
  };
  State state_ = State::kHunt;
  uint8_t type_ = 0;
  uint16_t len_ = 0, pos_ = 0, crc_ = 0xFFFF, crcRx_ = 0;
  uint8_t buf_[FRAME_MAX_PAYLOAD];

  void reset() {
    state_ = State::kHunt;
    type_ = 0;
    len_ = pos_ = crcRx_ = 0;
    crc_ = 0xFFFF;
  }
};
