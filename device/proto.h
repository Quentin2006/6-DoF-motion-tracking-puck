#pragma once

#include <stdint.h>
#include <stddef.h>

struct Quat {
  float x;
  float y;
  float z;
  float w;
};

constexpr size_t QUAT_PACKET_SIZE = 16;
static_assert(sizeof(Quat) == 16, "Quat wire format must be 16 bytes");
