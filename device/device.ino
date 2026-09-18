#include <Arduino.h> // first: defines RGB_BUILTIN, used by config.h
#include "config.h"
#include "src/position/tracker.h"
#include "packet.h"
#include "link.h"
#include "src/display/display.h"
#include "src/display/video.h"

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
  if (!tracker.boot()) {
    halt("MPU6050 init failed");
  }
  display.begin();
  rgbLedWrite(RGB_LED_PIN, 0, 255, 0); // calibrated, streaming
}

void loop() {
  if (tracker.step()) {
    usbLink.sendQuat(tracker.quat(), tracker.gyro());
  }
  // Video: budgeted multi-chunk drain (never drain-all — it starves 500 Hz
  // fusion). vidBuf must be 4-byte aligned: Display::chunkInner ->
  // writePixels SPI DMA reads 16-bit words, and a misaligned stack byte
  // buffer corrupts on word boundaries (periodic per-pixel pattern).
  alignas(4) static uint16_t vidBuf[VIDEO_PAYLOAD_MAX / 2];
  const unsigned long t0 = micros();
  int n = 0;
  bool open = false;
  while (n < VIDEO_DRAIN_MAX_CHUNKS && (unsigned long)(micros() - t0) < VIDEO_DRAIN_BUDGET_US) {
    VideoChunkHdr hdr = {};
    const int got = usbLink.pollVideoChunk((uint8_t *)vidBuf, hdr);
    if (got <= 0) {
      break;
    }
    if (!open) {
      display.beginBurst();
      open = true;
    }
    display.chunkInner(hdr, vidBuf, got);
    n++;
  }
  if (open) {
    display.endBurst();
  }
  tracker.report(usbLink.sequence());
  display.tick(millis());
}
