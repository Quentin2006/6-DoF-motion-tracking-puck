#include <Arduino.h> // first: Serial/millis + the FreeRTOS task API

#include "video_task.h"

#include "../../config.h"
#include "display.h"
#include "frame.h"
#include "video.h"
#include <JPEGDEC.h>
#include <cstring>

namespace {

// Cross-core stats: the loop core reads, this task writes. A torn 32-bit read
// at the 1 Hz boundary is harmless — these are rates, not accounting.
volatile uint32_t gFrames = 0, gDrops = 0, gBad = 0, gBytes = 0;

Display *gDisplay = nullptr;
SemaphoreHandle_t gSerialReady = nullptr;
volatile bool gGo = false;

// All of JPEGDEC's state (including its pixel block buffer) lives in the
// object, and the assembler holds the JPEG bytes: no heap, no per-frame
// allocation, nothing to close after openRAM (close() only closes a file
// handle, and there is none).
JPEGDEC gJpeg;
VideoFragAssembler gAsm;
uint32_t gAsmDropped = 0;

// What the panel is showing right now (RGB565, row-major 320x240). A block is
// only blitted when it differs from this, which is what keeps the paint rate
// above the frame rate: the SPI path tops out at ~3.4 MB/s (measured; the
// Adafruit/ESP32 pixel write is a 64 B polling loop), so a full-frame blit
// costs ~45 ms and would cap the panel at ~18 fps on its own.
alignas(4) uint16_t gPanel[VIDEO_W * VIDEO_H];
bool gPanelValid = false;

// JPEGDEC hands out MCU-aligned blocks (<= 2048 px, so ~45 per frame at
// 320x240 4:2:0); each one is blitted as its own window. The blit is
// contiguous, so this relies on iWidthUsed == iWidth — true for the geometry
// paint() enforces (320x240 is a whole number of 16x16 MCUs, so every block is
// a clean rectangle).
int drawBlock(JPEGDRAW *p) {
  const int w = p->iWidthUsed, h = p->iHeight;
  bool changed = !gPanelValid;
  for (int y = 0; y < h && !changed; y++) {
    const uint16_t *row = gPanel + (size_t)(p->y + y) * VIDEO_W + p->x;
    changed = memcmp(p->pPixels + (size_t)y * p->iWidth, row, (size_t)w * 2) != 0;
  }
  if (!changed) {
    return 1;
  }
  gDisplay->blitBlock(p->x, p->y, w, h, p->pPixels);
  for (int y = 0; y < h; y++) {
    uint16_t *row = gPanel + (size_t)(p->y + y) * VIDEO_W + p->x;
    memcpy(row, p->pPixels + (size_t)y * p->iWidth, (size_t)w * 2);
  }
  gPanelValid = true; // every blitted block is tracked from here on
  return 1;
}

// Decode one assembled JPEG straight into the panel, inside a single SPI
// transaction. Returns false for anything the panel cannot show.
bool paint(const uint8_t *jpeg, size_t len) {
  if (gJpeg.openRAM(const_cast<uint8_t *>(jpeg), (int)len, drawBlock) == 0) {
    return false;
  }
  // The blit assumes whole MCU blocks: 320x240 is exactly 20x15 16x16 MCUs for
  // the 4:2:0 the sender emits, so iWidthUsed == iWidth everywhere and every
  // block is a clean rectangle. Anything else is a sender bug, not a frame.
  if (gJpeg.getWidth() != VIDEO_W || gJpeg.getHeight() != VIDEO_H) {
    return false;
  }
  gJpeg.setPixelType(RGB565_LITTLE_ENDIAN); // matches Display::blitBlock's word reads
  // A frame arrived even if nothing needs repainting: that, not the blit, is
  // what must keep the idle RGB cycle away.
  gDisplay->markVideo(millis());
  gDisplay->beginBurst();
  const int ok = gJpeg.decode(0, 0, 0);
  gDisplay->endBurst();
  return ok != 0;
}

void videoTaskEntry(void *) {
  uint8_t scratch[1024];
  uint8_t payload[FRAME_MAX_PAYLOAD];
  FrameParser parser;

  // CDC init happens here so its RX interrupt lands on this core (see the
  // header). Grow the RX queue first, or HWCDC::begin creates its 256 B
  // default and a burst overflows it. A failed grow leaves the link unable to
  // carry a frame, so fail-stop like setup()'s MPU halt.
  if (Serial.setRxBufferSize(CDC_RX_BYTES) != CDC_RX_BYTES) {
    while (true) {
      delay(1000);
    }
  }
  Serial.begin(CDC_BAUD);
  xSemaphoreGive(gSerialReady);

  // boot() owns Serial until videoTaskGo() (it reads the CAL6 tokens).
  while (!gGo) {
    vTaskDelay(1);
  }

  for (;;) {
    const size_t n = Serial.read(scratch, sizeof scratch);
    if (n == 0) {
      // Nothing queued: yield the core (the CDC ISR and the idle task need
      // it) and let the panel fall back to its RGB cycle — tick self-gates on
      // VIDEO_IDLE_MS, so calling it every idle millisecond is free.
      vTaskDelay(1);
      if (gDisplay->tick(millis())) {
        gPanelValid = false;
      }
      continue;
    }
    gBytes += n;
    for (size_t i = 0; i < n; i++) {
      uint8_t type = 0;
      uint16_t len = 0;
      if (!parser.push(scratch[i], type, payload, len)) {
        continue;
      }
      if (type != FRAME_VIDEO) {
        continue;
      }
      VideoFragHdr hdr = {};
      if (!parse_video_frag_hdr(payload, len, hdr)) {
        continue;
      }
      if (!gAsm.push(hdr, payload + VIDEO_HDR_LEN, hdr.len)) {
        continue;
      }
      // A whole JPEG arrived: decode and paint it before reading on. The RX
      // queue is sized to hold the next frame's fragments while this runs.
      if (paint(gAsm.buf(), gAsm.len())) {
        gFrames++;
      } else {
        gBad++;
      }
    }
    const uint32_t dropped = gAsm.dropped();
    gDrops += dropped - gAsmDropped;
    gAsmDropped = dropped;
    // The idle RGB cycle repaints the panel behind our back: whatever the
    // shadow thinks is on screen is wrong from then on.
    if (gDisplay->tick(millis())) {
      gPanelValid = false;
    }
  }
}

} // namespace

void videoTaskStart(Display &display) {
  gDisplay = &display;
  gSerialReady = xSemaphoreCreateBinary();
  if (xTaskCreatePinnedToCore(videoTaskEntry, "video", VIDEO_TASK_STACK, nullptr, VIDEO_TASK_PRIO,
                              nullptr, VIDEO_TASK_CORE) != pdPASS) {
    // Fail-stop, like setup()'s MPU halt: nothing can report this yet, because
    // the task that was going to initialize Serial is the one that failed.
    while (true) {
      delay(1000);
    }
  }
  xSemaphoreTake(gSerialReady, portMAX_DELAY);
}

void videoTaskGo() { gGo = true; }

VideoStats videoTakeStats() {
  VideoStats s;
  s.frames = gFrames;
  s.drops = gDrops;
  s.bad = gBad;
  s.kb = gBytes >> 10;
  gFrames = gDrops = gBad = 0;
  gBytes -= s.kb * 1024; // keep the sub-KiB remainder across intervals
  return s;
}
