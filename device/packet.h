#pragma once
#include "wire.h"
#include <cstddef>
#include <cstdint>
#include <cstring>

// Wire layout (QUAT_PACKET_LEN bytes), little-endian. Consumed by
// engine/src/tracking.rs and asserted against tests/packet_fixture.txt.
//   seq u32 | stamp u32 | quat x,y,z,w f32 | gyro x,y,z f32
// Quat order is x,y,z,w to match Quat::from_xyzw on the engine side. QUAT_PACKET_LEN
// itself is generated into wire.h from tools/wire_spec.json.

// Build a pose packet into `pkt` (must be >= QUAT_PACKET_LEN bytes).
// Deliberately host-compilable (no Arduino/WiFi): the raw byte contract in one
// place, so the firmware producer and the C++ host test share the exact layout.
inline size_t build_quat_packet(uint8_t *pkt, uint32_t seq, uint32_t stamp,
                                const float quat[4], const float gyro[3]) {
  memcpy(pkt + 0, &seq, 4);
  memcpy(pkt + 4, &stamp, 4);
  memcpy(pkt + 8, &quat[0], 4);
  memcpy(pkt + 12, &quat[1], 4);
  memcpy(pkt + 16, &quat[2], 4);
  memcpy(pkt + 20, &quat[3], 4);
  memcpy(pkt + 24, &gyro[0], 4);
  memcpy(pkt + 28, &gyro[1], 4);
  memcpy(pkt + 32, &gyro[2], 4);
  return QUAT_PACKET_LEN;
}
