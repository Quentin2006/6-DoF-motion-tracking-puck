#pragma once

#include <stdint.h>
#include <stddef.h>

typedef struct {
  float x;
  float y;
  float z;
  float w;
} Quat;

constexpr size_t QUAT_PACKET_SIZE = 16;
static_assert(sizeof(Quat) == 16, "Quat wire format must be 16 bytes");
