#pragma once

#include <cstddef>
#include <cstdint>

struct BulkUsbStats {
  uint32_t rxBytes = 0;
  uint32_t rxOverflowBytes = 0;
  uint16_t rxQueueHighWaterChunks = 0;
};

// One TinyUSB vendor interface with a 64-byte duplex bulk endpoint pair.
// tinyusb_enable_interface must run before USB.begin() (which begin() itself
// calls); all application bytes share this pipe.
class BulkUsb {
public:
  static bool begin();
  static size_t read(uint8_t *dst, size_t cap);
  static bool try_write(const uint8_t *src, size_t len);
  static BulkUsbStats takeStats();
};
