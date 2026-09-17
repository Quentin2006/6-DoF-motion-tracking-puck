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
  // vidBuf must be 4-byte aligned: Display::chunk -> writePixels SPI DMA
  // reads 16-bit words, and a misaligned stack byte buffer corrupts on word
  // boundaries (periodic per-pixel pattern).
  alignas(4) static uint16_t vidBuf[VIDEO_PAYLOAD_MAX / 2];
  VideoChunkHdr hdr = {};
  const int n = usbLink.pollVideoChunk((uint8_t *)vidBuf, hdr);
  display.chunk(hdr, vidBuf, n); // n <= 0: no-op
  tracker.report(usbLink.sequence());
  display.tick(millis());
}
