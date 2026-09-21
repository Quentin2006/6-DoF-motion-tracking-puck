#pragma once
#include <cstdint>

class Display;

// Video RX + panel blit (device/src/display/video_task.cpp), owned by one
// core-0 task for the life of the firmware. After videoTaskStart() nothing
// else in the sketch may read Serial: the task is the only consumer of the
// CDC RX queue, and the loop core must stay free for the 500 Hz fusion tick.
struct VideoStats {
  uint32_t frames; // video chunks painted since the last call
  uint32_t drops;  // reserved for whole-frame loss (0 today)
  uint32_t bad;    // reserved for rejected frames (0 today)
  uint32_t kb;     // Serial RX bytes since the last call, in KiB
};

// Start the task and hand it the panel it blits through. Call after
// Display::begin() and after Tracker::boot() returns (boot() reads Serial for
// the CAL6 tokens; two readers would steal each other's bytes).
void videoTaskStart(Display &display);

// Read-and-zero the counters: per-interval deltas, printed once a second.
VideoStats videoTakeStats();
