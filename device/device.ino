#include <Arduino.h> // first: defines RGB_BUILTIN, used by config.h
#include "config.h"
#include "src/position/tracker.h"
#include "packet.h"
#include "link.h"
#include "src/display/display.h"
#include "src/display/video_task.h"

Tracker tracker;
UsbLink usbLink;
Display display;

// Deterministic fail-stop: a puck with no IMU has nothing to do.
static void halt(const char *why) {
  Serial.println(why);
  while (true) {
    delay(1000);
  }
}

void setup() {
  // Serial — and with it the CDC RX interrupt — belongs to the video task
  // from here: it initializes the CDC on core 0, so the per-byte RX cost
  // never lands on the fusion core. Nothing else may read Serial afterwards.
  videoTaskStart(display);
  delay(500);
  rgbLedWrite(RGB_LED_PIN, 255, 0, 0); // calibrating / not ready
  if (!tracker.boot()) {
    halt("MPU6050 init failed");
  }
  display.begin();
  videoTaskGo(); // boot() is done with Serial; the panel is up
  rgbLedWrite(RGB_LED_PIN, 0, 255, 0); // calibrated, streaming
}

void loop() {
  if (tracker.step()) {
    usbLink.sendQuat(tracker.quat(), tracker.gyro());
  }
  tracker.report(usbLink.sequence());
  // Video counters: independent 1 Hz gate (tracker.report has its own).
  static unsigned long lastVidMs = 0;
  if ((unsigned long)(millis() - lastVidMs) >= 1000UL) {
    lastVidMs = millis();
    const VideoStats v = videoTakeStats();
    Serial.printf("vid %lu drop %lu bad %lu kb %lu\n", (unsigned long)v.frames,
                  (unsigned long)v.drops, (unsigned long)v.bad, (unsigned long)v.kb);
  }
}
