#pragma once

#include <Arduino.h>

#include <cstddef>
#include <cstdint>

struct BulkUsbStats {
  uint32_t rxBytes = 0;
  uint32_t rxOverflowBytes = 0;
  uint32_t txBytes = 0;
  uint32_t txBackpressureDrops = 0;
  uint32_t txWriteErrors = 0;
  uint16_t rxQueueHighWaterChunks = 0;
};

// One TinyUSB vendor interface with a 64-byte duplex bulk endpoint pair.
// begin() must run before USB.begin(); all application bytes share this pipe.
class BulkUsb {
public:
  static bool begin();
  static bool mounted();
  static size_t read(uint8_t *dst, size_t cap);
  static size_t write_available();
  static bool try_write(const uint8_t *src, size_t len);
  static BulkUsbStats takeStats();
};
