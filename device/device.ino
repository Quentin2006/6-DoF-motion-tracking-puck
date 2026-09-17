#include <Arduino.h> // first: defines RGB_BUILTIN, used by config.h
#include "config.h"
#include "tracker.h"
#include "packet.h"
#include "link.h"
#include "display.h"
#include "video.h"

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
  usbLink.begin(); // Serial + 4 KB RX buffer; never fails
  delay(500);
  rgbLedWrite(RGB_LED_PIN, 255, 0, 0); // calibrating / not ready
  if (!tracker.begin()) {
    halt("MPU6050 init failed");
  }
  delay(500);
  Serial.println("calibrating...");
  tracker.calibrate();
  Serial.println("calibrated");
  tracker.printAccelCal();
  Serial.println("send CAL6 within 5 s for accel cal");
  tracker.maybeRunAccelCal(5000UL);
  tracker.start();
  display.begin();
  rgbLedWrite(RGB_LED_PIN, 0, 255, 0); // calibrated, streaming
}

void loop() {
  if (tracker.step()) {
    usbLink.sendQuat(tracker.quat(), tracker.gyro());
  }
  // Video: one chunk per loop max (never a drain — it starves 500 Hz fusion).
  VideoChunkHdr hdr;
  uint8_t buf[VIDEO_PAYLOAD_MAX];
  const int n = usbLink.pollVideoChunk(buf, hdr);
  display.chunk(hdr, (const uint16_t *)buf, n); // n <= 0: no-op
  tracker.report(usbLink.sequence());
  display.tick(millis());
}
