#include <Arduino.h> // first: defines RGB_BUILTIN, used by config.h
#include "config.h"
#include "src/position/tracker.h"
#include "link.h"
#include "src/display/display.h"
#include "src/display/video_task.h"

Tracker tracker;
BulkLink bulkLink;
Display display;

// Deterministic fail-stop: a puck with no IMU has nothing to do.
static void halt(const char *why) {
  Serial0.println(why);
  while (true) {
    delay(1000);
  }
}

void setup() {
  Serial0.begin(CONSOLE_BAUD);
  // Initialize the vendor-bulk endpoint on the video task's core before the
  // tracker enters its UART0 calibration window.
  if (!videoTaskStart(display)) {
    halt("USB bulk init failed");
  }
  delay(500);
  rgbLedWrite(RGB_LED_PIN, 255, 0, 0); // calibrating / not ready
  if (!tracker.boot()) {
    halt("MPU6050 init failed");
  }
  display.begin();
  videoTaskGo(); // UART0 calibration is done and the panel is ready
  rgbLedWrite(RGB_LED_PIN, 0, 255, 0); // calibrated, streaming
}

void loop() {
  if (tracker.step()) {
    bulkLink.sendQuat(tracker.quat(), tracker.gyro());
  }
  tracker.report(bulkLink.sequence());
  // Video counters: independent 1 Hz gate (tracker.report has its own).
  static unsigned long lastVidMs = 0;
  if ((unsigned long)(millis() - lastVidMs) >= VIDEO_COUNTER_PERIOD_MS) {
    lastVidMs = millis();
    const VideoStats v = videoTakeStats();
    Serial0.printf("vid %lu drop %lu bad %lu kb %lu rxdrop %lu qhi %lu\n",
                   (unsigned long)v.frames, (unsigned long)v.drops, (unsigned long)v.bad,
                   (unsigned long)v.kb, (unsigned long)v.rxDrop, (unsigned long)v.qhi);
  }
}
