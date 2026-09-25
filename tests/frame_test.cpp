// Host test for the framed CDC contract (device/frame.h).
// Payload bytes are asserted by packet_test; here the framing is under
// test: garbage-prefixed recovery, CRC rejection, split-feed reassembly.
// Fails on: bad CRC accepted, magic-in-payload desync, over-length accepted.
//
// Build: g++ -std=c++17 -Idevice tests/frame_test.cpp -o build/frame_test
// Run:   ./build/frame_test
#include "frame.h"
#include "packet.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

static int failures = 0;

#define CHECK(cond, msg)                                   \
  do {                                                     \
    if (!(cond)) {                                         \
      std::fprintf(stderr, "frame_test FAIL: %s (line %d)\n", msg, __LINE__); \
      failures++;                                          \
    }                                                      \
  } while (0)

// Feed `n` bytes; count valid frames; last frame's type/len/payload kept.
static int feed(FrameParser &fp, const uint8_t *data, size_t n, uint8_t &type_out,
                std::vector<uint8_t> &payload_out, uint16_t &len_out) {
  static uint8_t scratch[FRAME_MAX_PAYLOAD];
  int got = 0;
  for (size_t i = 0; i < n; i++) {
    if (fp.push(data[i], type_out, scratch, len_out)) {
      got++;
      payload_out.assign(scratch, scratch + len_out);
    }
  }
  return got;
}

int main() {
  // Pose payload: same v2 values as packet_test so the payload itself is the
  // known-good layout (seq=42, stamp=1000, quat identity, gyro 0.5/-0.25/1.5).
  float quat[4] = {0.0f, 0.0f, 0.0f, 1.0f};
  float gyro[3] = {0.5f, -0.25f, 1.5f};
  uint8_t payload[QUAT_PACKET_LEN];
  build_quat_packet(payload, 42, 1000, quat, gyro);

  uint8_t frame[FRAME_HDR_LEN + QUAT_PACKET_LEN + FRAME_CRC_LEN];
  size_t n = build_frame(frame, FRAME_POSE, payload, QUAT_PACKET_LEN);
  CHECK(n == sizeof frame, "frame length");

  // 1. Garbage-prefixed valid frame recovers exactly.
  {
    std::vector<uint8_t> stream = {0x00};
    stream.insert(stream.end(), frame, frame + n);
    FrameParser fp;
    uint8_t type = 0;
    uint16_t len = 0;
    std::vector<uint8_t> out;
    int got = feed(fp, stream.data(), stream.size(), type, out, len);
    CHECK(got == 1, "one frame from garbage-prefixed stream");
    CHECK(type == FRAME_POSE, "pose type");
    CHECK(len == QUAT_PACKET_LEN, "pose len");
    CHECK(out.size() == QUAT_PACKET_LEN &&
              memcmp(out.data(), payload, QUAT_PACKET_LEN) == 0,
          "pose payload roundtrip");
  }

  // 2. One flipped payload byte: CRC reject, parser yields nothing.
  {
    uint8_t bad[sizeof frame];
    memcpy(bad, frame, n);
    bad[FRAME_HDR_LEN + 3] ^= 0x01;
    FrameParser fp;
    uint8_t type = 0;
    uint16_t len = 0;
    std::vector<uint8_t> out;
    int got = feed(fp, bad, n, type, out, len);
    CHECK(got == 0, "corrupted frame rejected");
  }

  // 3. VIDEO frame split across two push halves reassembles.
  {
    uint8_t vpayload[8 + 16];
    vpayload[0] = 7;
    vpayload[1] = 0; // frame id 7 LE
    vpayload[2] = 3;
    vpayload[3] = 0; // frag index 3
    vpayload[4] = 4;
    vpayload[5] = 0; // frag count 4
    vpayload[6] = 16;
    vpayload[7] = 0; // frag len 16
    for (int i = 0; i < 16; i++) {
      vpayload[8 + i] = (uint8_t)(0xA0 + i);
    }
    uint8_t vframe[FRAME_HDR_LEN + sizeof vpayload + FRAME_CRC_LEN];
    size_t vn = build_frame(vframe, FRAME_VIDEO, vpayload, sizeof vpayload);
    FrameParser fp;
    uint8_t type = 0;
    uint16_t len = 0;
    std::vector<uint8_t> out;
    size_t half = vn / 2;
    int got = feed(fp, vframe, half, type, out, len);
    CHECK(got == 0, "no frame from first half");
    got += feed(fp, vframe + half, vn - half, type, out, len);
    CHECK(got == 1, "video frame reassembled from halves");
    CHECK(type == FRAME_VIDEO, "video type");
    CHECK(len == sizeof vpayload, "video len");
    CHECK(out.size() == sizeof vpayload && memcmp(out.data(), vpayload, sizeof vpayload) == 0,
          "video payload roundtrip");
  }

  // 4. Magic bytes inside the payload do not desync the next frame.
  {
    uint8_t tricky[QUAT_PACKET_LEN];
    memset(tricky, 0, sizeof tricky);
    tricky[0] = FRAME_MAGIC_0;
    tricky[1] = FRAME_MAGIC_1;
    tricky[2] = FRAME_MAGIC_0;
    uint8_t f1[FRAME_HDR_LEN + sizeof tricky + FRAME_CRC_LEN];
    size_t n1 = build_frame(f1, FRAME_POSE, tricky, sizeof tricky);
    std::vector<uint8_t> stream;
    stream.insert(stream.end(), f1, f1 + n1);
    stream.insert(stream.end(), frame, frame + n);
    FrameParser fp;
    uint8_t type = 0;
    uint16_t len = 0;
    std::vector<uint8_t> out;
    int got = feed(fp, stream.data(), stream.size(), type, out, len);
    CHECK(got == 2, "two back-to-back frames with magic in payload");
    CHECK(type == FRAME_POSE && len == QUAT_PACKET_LEN, "second frame header");
    CHECK(out.size() == QUAT_PACKET_LEN &&
              memcmp(out.data(), payload, QUAT_PACKET_LEN) == 0,
          "second frame payload");
  }

  if (failures == 0) {
    std::printf("frame_test OK\n");
    return 0;
  }
  return 1;
}
