#include "usb_bulk.h"

#include "../../config.h"
#include "USB.h"
#include "freertos/task.h"
#include "esp32-hal-tinyusb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <algorithm>
#include <cstring>
#include <atomic>

namespace {
constexpr size_t kChunkSize = 64;
constexpr UBaseType_t kQueueDepth = 512;

struct RxChunk {
  uint8_t len;
  uint8_t data[kChunkSize];
};

StaticQueue_t sQueueControl;
uint8_t sQueueStorage[kQueueDepth * sizeof(RxChunk)] alignas(8);
QueueHandle_t sQueue = nullptr;
bool sBegun = false;

uint8_t sReadRemainder[kChunkSize];
uint8_t sReadRemainderLen = 0;
std::atomic<int32_t> sRxQueueOccupancy{0};
std::atomic<uint16_t> sRxQueueHighWaterChunks{0};

volatile uint32_t sRxBytes = 0;
volatile uint32_t sRxOverflowBytes = 0;

// The only mount check in the class: TinyUSB reports zero available bytes while
// the vendor interface is down, so a not-mounted device cannot be written.
static bool mounted() { return sBegun && tud_vendor_n_mounted(0); }

BulkUsbStats sLastStats;

uint16_t load_bulk_descriptor(uint8_t *dst, uint8_t *itf) {
  const uint8_t ep = tinyusb_get_free_duplex_endpoint();
  if (ep == 0) {
    return 0;
  }
  const uint8_t str = tinyusb_add_string_descriptor("6-DoF Motion Tracking Bulk");
  const uint8_t descriptor[TUD_VENDOR_DESC_LEN] = {
      TUD_VENDOR_DESCRIPTOR(*itf, str, ep, static_cast<uint8_t>(0x80 | ep), kChunkSize)};
  ++*itf;
  std::memcpy(dst, descriptor, sizeof(descriptor));
  return sizeof(descriptor);
}
} // namespace

static void accept_vendor_rx(uint8_t itf) {
  if (itf != 0) {
    return;
  }

  RxChunk chunk = {};
  for (;;) {
    // TinyUSB's task has higher priority than the core-0 consumer. Leave one
    // queue slot and wait for the consumer before accepting another chunk, so
    // the USB FIFO applies backpressure instead of losing accepted bytes.
    while (uxQueueMessagesWaiting(sQueue) >= kQueueDepth - 1) {
      vTaskDelay(1);
    }
    const uint32_t available = tud_vendor_n_available(0);
    if (available == 0) {
      break;
    }
    const uint32_t wanted = std::min<uint32_t>(available, kChunkSize);
    const uint32_t received = tud_vendor_n_read(0, chunk.data, wanted);
    if (received == 0) {
      break;
    }
    chunk.len = static_cast<uint8_t>(received);
    // Reserve before enqueue so a consumer cannot dequeue between the queue
    // operation and the high-water sample and under-report the peak.
    const int32_t occupancy =
        sRxQueueOccupancy.fetch_add(1, std::memory_order_relaxed) + 1;
    if (xQueueSend(sQueue, &chunk, 0) == pdTRUE) {
      uint16_t peak = sRxQueueHighWaterChunks.load(std::memory_order_relaxed);
      while (occupancy > peak &&
             !sRxQueueHighWaterChunks.compare_exchange_weak(
                 peak, static_cast<uint16_t>(occupancy), std::memory_order_relaxed)) {
      }
      sRxBytes += received;
    } else {
      sRxQueueOccupancy.fetch_sub(1, std::memory_order_relaxed);
      sRxOverflowBytes += received;
      vTaskDelay(1);
    }
  }
}

// TinyUSB 0.19+ passes the interface plus callback data. Buffered vendor mode
// supplies no data here; accept_vendor_rx drains the RX FIFO itself.
extern "C" void tud_vendor_rx_cb(uint8_t itf, const uint8_t *, uint32_t) {
  accept_vendor_rx(itf);
}

bool BulkUsb::begin() {
  if (sBegun) {
    return true;
  }
  sQueue = xQueueCreateStatic(kQueueDepth, sizeof(RxChunk), sQueueStorage, &sQueueControl);
  if (sQueue == nullptr) {
    return false;
  }

  if (!USB.VID(PUCK_USB_VID) || !USB.PID(PUCK_USB_PID) ||
      !USB.manufacturerName("6-DoF Motion Tracking") ||
      !USB.productName("6-DoF Motion Tracking Puck") || !USB.firmwareVersion(0x0100)) {
    return false;
  }
  if (tinyusb_enable_interface(USB_INTERFACE_VENDOR, TUD_VENDOR_DESC_LEN, load_bulk_descriptor) != ESP_OK) {
    return false;
  }
  if (!USB.begin()) {
    return false;
  }

  sBegun = true;
  return true;
}

size_t BulkUsb::read(uint8_t *dst, size_t cap) {
  if (!sBegun || dst == nullptr || cap == 0) {
    return 0;
  }

  size_t total = 0;
  while (total < cap) {
    if (sReadRemainderLen == 0) {
      RxChunk chunk;
      if (xQueueReceive(sQueue, &chunk, 0) != pdTRUE) {
        break;
      }
      sRxQueueOccupancy.fetch_sub(1, std::memory_order_relaxed);
      sReadRemainderLen = chunk.len;
      std::memcpy(sReadRemainder, chunk.data, chunk.len);
    }

    const size_t take = std::min(cap - total, static_cast<size_t>(sReadRemainderLen));
    std::memcpy(dst + total, sReadRemainder, take);
    total += take;
    sReadRemainderLen -= static_cast<uint8_t>(take);
    if (sReadRemainderLen != 0) {
      std::memmove(sReadRemainder, sReadRemainder + take, sReadRemainderLen);
    }
  }
  return total;
}

bool BulkUsb::try_write(const uint8_t *src, size_t len) {
  if (src == nullptr || len == 0 || !mounted() || tud_vendor_n_write_available(0) < len) {
    return false;
  }

  const uint32_t written = tud_vendor_n_write(0, src, len);
  if (written != len) {
    tud_vendor_n_write_clear(0);
    return false;
  }
  const uint32_t flushed = tud_vendor_n_write_flush(0);
  // TinyUSB may report zero while the IN endpoint is busy; the complete frame
  // is already buffered and TinyUSB will auto-flush it. Any non-zero short
  // flush is a real partial-frame failure.
  if (flushed != 0 && flushed != len) {
    return false;
  }
  return true;
}

BulkUsbStats BulkUsb::takeStats() {
  const BulkUsbStats current{
      sRxBytes, sRxOverflowBytes,
      sRxQueueHighWaterChunks.load(std::memory_order_relaxed)};
  const BulkUsbStats delta{
      current.rxBytes - sLastStats.rxBytes,
      current.rxOverflowBytes - sLastStats.rxOverflowBytes,
      current.rxQueueHighWaterChunks,
  };
  sLastStats = current;
  return delta;
}
