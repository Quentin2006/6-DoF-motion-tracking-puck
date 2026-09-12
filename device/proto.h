#pragma once

#include <stdint.h>
#include <stddef.h>

struct Quat {
  float x;
  float y;
  float z;
  float w;
};

struct QuatPacket {
  uint32_t seq;
  uint32_t micros;
  Quat quat;
};

constexpr size_t QUAT_PACKET_SIZE = 24;
static_assert(sizeof(QuatPacket) == 24, "wire must be 24 bytes");
