#pragma once
#include <cstdint>

class Display;

// Video RX + panel blit (device/src/display/video_task.cpp), owned by one
// core-0 task for the life of the firmware. The task owns Serial end to end:
// after videoTaskStart() nothing else in the sketch may read it or call
// Serial.begin().
struct VideoStats {
  uint32_t frames; // video chunks painted since the last call
  uint32_t drops;  // reserved for whole-frame loss (0 today)
  uint32_t bad;    // reserved for rejected frames (0 today)
  uint32_t kb;     // Serial RX bytes since the last call, in KiB
};

// Start the task, hand it the panel it will blit through, and initialize the
// CDC from core 0. Returns once Serial is up.
//
// Core 0 is the point: HWCDC registers its RX interrupt with the core that
// calls Serial.begin(), and that handler pushes one FreeRTOS queue item per
// received byte (~400k/s at link saturation). Registered from the loop core
// (core 1) it costs the 500 Hz fusion tick ~20%; registered from core 0 it
// costs the fusion loop nothing.
//
// The task then parks until videoTaskGo(): Tracker::boot() still reads Serial
// for the CAL6 tokens, and the panel is not up yet — nothing touches
// `display` before then.
void videoTaskStart(Display &display);

// Start reading Serial and blitting. Call after Tracker::boot() has returned
// and Display::begin() has run.
void videoTaskGo();

// Read-and-zero the counters: per-interval deltas, printed once a second.
VideoStats videoTakeStats();
