// Host test for the v2 packet builder (device/packet.h). This is the producer
// half of the wire contract; engine/src/tracking.rs is the consumer half. Both
// read tests/packet_fixture.txt — the single source of truth for the 36-byte
// little-endian layout. If either half changes the format, its own test fails
// against this fixture.
//
// Build: g++ -std=c++17 -Idevice tests/packet_test.cpp -o build/packet_test
// Run:   ./build/packet_test tests/packet_fixture.txt
#include "packet.h"
#include <cctype>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

static int hex_val(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static std::string fixture_hex(const std::string &path) {
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    std::string t;
    for (char c : line)
      if (!isspace((unsigned char)c)) t += (char)tolower((unsigned char)c);
    if (t.empty() || t[0] == '#') continue;
    return t;
  }
  return "";
}

static std::vector<uint8_t> hex_bytes(const std::string &s) {
  std::vector<uint8_t> out;
  for (size_t i = 0; i + 1 < s.size(); i += 2) {
    int hi = hex_val(s[i]), lo = hex_val(s[i + 1]);
    if (hi < 0 || lo < 0) return {};
    out.push_back((uint8_t)((hi << 4) | lo));
  }
  return out;
}

int main(int argc, char **argv) {
  std::string path = argc > 1 ? argv[1] : "tests/packet_fixture.txt";
  std::string hex = fixture_hex(path);
  if (hex.empty()) {
    std::fprintf(stderr, "packet_test: no fixture found at %s\n", path.c_str());
    return 1;
  }

  uint32_t seq = 42, stamp = 1000;
  float quat[4] = {0.0f, 0.0f, 0.0f, 1.0f};
  float gyro[3] = {0.5f, -0.25f, 1.5f};

  uint8_t pkt[QUAT_PACKET_LEN_V2];
  size_t n = build_quat_packet(pkt, seq, stamp, quat, gyro);
  if (n != QUAT_PACKET_LEN_V2) {
    std::fprintf(stderr, "packet_test: bad length %zu\n", n);
    return 1;
  }

  const std::vector<uint8_t> want = hex_bytes(hex);
  const std::vector<uint8_t> got(pkt, pkt + n);
  if (got != want) {
    std::fprintf(stderr, "packet_test: MISMATCH (%zu bytes)\n", n);
    return 1;
  }

  std::printf("packet_test OK (%zu bytes)\n", n);
  return 0;
}
