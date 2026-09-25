#pragma once
#include <cstdint>

class Display;

// Video bulk RX + JPEG decode + panel blit, owned by one core-0 task. It is
// the sole BulkUsb consumer; UART0 remains independent for console I/O.
struct VideoStats {
  uint32_t frames; // completed blobs that painted since the last call (one per frame, not per JPEG)
  uint32_t drops;  // fragments the assembler rejected (a lost or misordered fragment counts once)
  uint32_t bad;    // frames the device refused to paint (rect table or decode)
  uint32_t kb;     // accepted bulk RX bytes since the last call, in KiB
  uint32_t rxDrop; // bulk RX bytes dropped because the chunk queue was full
  uint32_t qhi;    // maximum queued 64-byte chunks since boot
};

// Start the task, hand it the panel it will blit through, and initialize the
// vendor-bulk endpoint. Returns true once transport initialization completes.
// Core 0 keeps JPEG decode and SPI painting away from the core-1 fusion loop.
// The task parks until videoTaskGo(): Tracker::boot() still owns UART0 for the
// CAL6 tokens, and the panel is not up yet.
bool videoTaskStart(Display &display);

// Start consuming bulk video and blitting after boot and Display::begin().
void videoTaskGo();

// Read-and-zero the counters: per-interval deltas, printed once a second.
VideoStats videoTakeStats();
