// Host test for the video fragment contract (device/src/display/video.h).
// Two things are under test: the pinned wire bytes of one fragment (produced by
// send_jpeg in engine/src/tracking.rs) and the assembler that puts a frame back
// together on the device.
// Fails on: header field order/endianness, CRC over the wrong span, a lost
// fragment spliced into a frame, out-of-order fragments accepted, or an
// over-long frame writing past the assembly buffer.
//
// ASAN is on purpose: "aborts without writing out of bounds" is only a real
// assertion with a redzone after the assembly buffer.
//
// Build: g++ -std=c++17 -fsanitize=address -Idevice tests/video_test.cpp -o build/video_test
// Run:   ./build/video_test
#include "frame.h"
#include "src/display/video.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

static int failures = 0;

#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "video_test FAIL: %s (line %d)\n", msg, __LINE__);   \
      failures++;                                                              \
    }                                                                          \
  } while (0)

// One fragment header + `data`.
static VideoFragHdr hdr(uint16_t frame, uint16_t frag, uint16_t count, uint16_t len) {
  VideoFragHdr h = {};
  h.frame = frame;
  h.frag = frag;
  h.count = count;
  h.len = len;
  return h;
}

static std::vector<uint8_t> bytes(size_t n, uint8_t seed) {
  std::vector<uint8_t> v(n);
  for (size_t i = 0; i < n; i++) {
    v[i] = (uint8_t)(seed + i);
  }
  return v;
}

int main() {
  // 1. One fragment's wire bytes, pinned to the literal the Rust producer test
  //    asserts (tracking.rs::video_fragment_wire_layout_is_pinned).
  {
    uint8_t payload[VIDEO_HDR_LEN + 3] = {0x34, 0x12, 0x00, 0x00, 0x02,
                                          0x00, 0x03, 0x00, 0xAA, 0xBB, 0xCC};
    uint8_t frame[FRAME_HDR_LEN + sizeof payload + FRAME_CRC_LEN];
    const size_t n = build_frame(frame, FRAME_VIDEO, payload, sizeof payload);
    const uint8_t want[FRAME_HDR_LEN + sizeof payload] = {
        0x55, 0xAA, 0x02, 0x0B, 0x00,                   // magic, FRAME_VIDEO, len 11
        0x34, 0x12, 0x00, 0x00, 0x02, 0x00, 0x03, 0x00, // frame/frag/count/len
        0xAA, 0xBB, 0xCC};                              // fragment bytes
    CHECK(n == sizeof frame, "fragment frame length");
    CHECK(memcmp(frame, want, sizeof want) == 0, "fragment header bytes");
    uint16_t crc = 0xFFFF;
    for (size_t i = 2; i < FRAME_HDR_LEN + sizeof payload; i++) {
      crc = frame_crc_feed(crc, frame[i]);
    }
    CHECK(frame[sizeof want] == (uint8_t)(crc & 0xFF) &&
              frame[sizeof want + 1] == (uint8_t)(crc >> 8),
          "fragment CRC over type+len+payload");
  }

  // 2. In-order fragments reassemble into exactly the blob that was split.
  {
    VideoFragAssembler asm_;
    const std::vector<uint8_t> a = bytes(VIDEO_FRAG_MAX, 0x10);
    const std::vector<uint8_t> b = bytes(VIDEO_FRAG_MAX, 0x80);
    const std::vector<uint8_t> c = bytes(17, 0xF0);
    CHECK(!asm_.push(hdr(5, 0, 3, VIDEO_FRAG_MAX), a.data(), a.size()), "frag 0 incomplete");
    CHECK(!asm_.push(hdr(5, 1, 3, VIDEO_FRAG_MAX), b.data(), b.size()), "frag 1 incomplete");
    CHECK(asm_.push(hdr(5, 2, 3, (uint16_t)c.size()), c.data(), c.size()), "frag 2 completes");
    CHECK(asm_.len() == a.size() + b.size() + c.size(), "assembled length");
    CHECK(memcmp(asm_.buf(), a.data(), a.size()) == 0, "assembled frag 0");
    CHECK(memcmp(asm_.buf() + a.size(), b.data(), b.size()) == 0, "assembled frag 1");
    CHECK(memcmp(asm_.buf() + a.size() + b.size(), c.data(), c.size()) == 0, "assembled frag 2");
    CHECK(asm_.dropped() == 0, "no drops");
    // A second frame right after the first starts clean.
    const std::vector<uint8_t> d = bytes(64, 0x40);
    CHECK(asm_.push(hdr(6, 0, 1, (uint16_t)d.size()), d.data(), d.size()), "single-fragment frame");
    CHECK(asm_.len() == d.size() && memcmp(asm_.buf(), d.data(), d.size()) == 0,
          "second frame replaces the first");
  }

  // 3. A gap abandons the frame, and the next frame still assembles: the cost
  //    of a lost fragment is exactly one frame, never a spliced image.
  {
    VideoFragAssembler asm_;
    const std::vector<uint8_t> a = bytes(100, 0x01);
    CHECK(!asm_.push(hdr(7, 0, 3, 100), a.data(), a.size()), "frag 0 accepted");
    CHECK(!asm_.push(hdr(7, 2, 3, 100), a.data(), a.size()), "frag 2 rejected");
    CHECK(asm_.dropped() == 1, "gap counted once");
    CHECK(asm_.len() == 0, "abandoned frame exposes no bytes");
    const std::vector<uint8_t> e = bytes(100, 0x22);
    CHECK(!asm_.push(hdr(8, 0, 2, 100), e.data(), e.size()), "next frame frag 0");
    CHECK(asm_.push(hdr(8, 1, 2, 100), e.data(), e.size()), "next frame completes");
    CHECK(asm_.len() == 200 && memcmp(asm_.buf(), e.data(), 100) == 0, "next frame intact");
    CHECK(asm_.dropped() == 1, "still one drop");
  }

  // 4. A frame id that changes mid-frame is rejected rather than spliced.
  {
    VideoFragAssembler asm_;
    const std::vector<uint8_t> a = bytes(100, 0x33);
    CHECK(!asm_.push(hdr(9, 0, 2, 100), a.data(), a.size()), "frag 0 accepted");
    CHECK(!asm_.push(hdr(10, 1, 2, 100), a.data(), a.size()), "other frame's frag 1 rejected");
    CHECK(asm_.dropped() == 1 && asm_.len() == 0, "mid-frame id change aborts");
  }

  // 5. Structurally impossible fragments: counted, never accepted.
  {
    VideoFragAssembler asm_;
    const std::vector<uint8_t> a = bytes(VIDEO_FRAG_MAX, 0x55);
    CHECK(!asm_.push(hdr(1, 0, 0, 100), a.data(), 100), "count 0 rejected");
    CHECK(!asm_.push(hdr(1, 0, 1, 0), a.data(), 100), "len 0 rejected");
    CHECK(!asm_.push(hdr(1, 0, 1, (uint16_t)(VIDEO_FRAG_MAX + 1)), a.data(), a.size()),
          "over-long fragment rejected");
    CHECK(!asm_.push(hdr(1, 1, 1, 100), a.data(), 100), "frag >= count rejected");
    CHECK(!asm_.push(hdr(1, 0, 1, 100), a.data(), 99), "short payload rejected");
    CHECK(asm_.dropped() == 5 && asm_.len() == 0, "all five counted, none assembled");
  }

  // 6. A frame that would outgrow the assembly buffer aborts at the boundary
  //    (ASAN redzone after buf_ makes any overflow here a failure).
  {
    VideoFragAssembler asm_;
    const std::vector<uint8_t> a = bytes(VIDEO_FRAG_MAX, 0x66);
    const int fits = (int)(VIDEO_JPEG_MAX / VIDEO_FRAG_MAX); // 38
    for (int i = 0; i < fits; i++) {
      CHECK(!asm_.push(hdr(2, (uint16_t)i, (uint16_t)(fits + 1), VIDEO_FRAG_MAX), a.data(), a.size()),
            "fragment within the cap");
    }
    CHECK(asm_.len() == (size_t)fits * VIDEO_FRAG_MAX, "bytes accepted up to the cap");
    CHECK(!asm_.push(hdr(2, (uint16_t)fits, (uint16_t)(fits + 1), VIDEO_FRAG_MAX), a.data(), a.size()),
          "fragment over the cap rejected");
    CHECK(asm_.dropped() == 1 && asm_.len() == 0, "over-cap frame abandoned");
    // Still usable: the next frame assembles.
    CHECK(asm_.push(hdr(3, 0, 1, (uint16_t)a.size()), a.data(), a.size()), "assembler survives");
    CHECK(asm_.len() == a.size(), "post-abort frame intact");
  }

  if (failures == 0) {
    std::printf("video_test OK\n");
    return 0;
  }
  return 1;
}
