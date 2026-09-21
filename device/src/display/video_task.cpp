#include <Arduino.h> // first: Serial/millis + the FreeRTOS task API

#include "video_task.h"

#include "../../config.h"
#include "display.h"
#include "frame.h"
#include "video.h"
#include <cstring>

namespace {

// Cross-core stats: the loop core reads, this task writes. A torn 32-bit read
// at the 1 Hz boundary is harmless — these are rates, not accounting.
volatile uint32_t gFrames = 0, gDrops = 0, gBad = 0, gBytes = 0;

Display *gDisplay = nullptr;
SemaphoreHandle_t gSerialReady = nullptr;
volatile bool gGo = false;

// One complete FRAME_VIDEO / FRAME_VIDEO_RLE payload -> the chunk's pixel
// window, keeping the old UsbLink::pollVideoChunk rules: a raw chunk must be
// exactly VIDEO_PAYLOAD_MAX bytes, an RLE chunk must decode to the whole
// chunk (decode failure drops it like a CRC reject). `pix` must be 4-byte
// aligned — Display::chunkInner -> writePixels reads it as 16-bit words.
bool chunkPixels(uint8_t type, const uint8_t *payload, uint16_t len, VideoChunkHdr &hdr,
                 uint16_t *pix) {
  if (!parse_video_header(payload, len, hdr) || hdr.len == 0 ||
      (size_t)hdr.len + VIDEO_HDR_LEN > len) {
    return false;
  }
  if (type == FRAME_VIDEO_RLE) {
    if (!rle_decode_565(payload + VIDEO_HDR_LEN, hdr.len, pix)) {
      return false;
    }
    hdr.len = VIDEO_PAYLOAD_MAX;
    return true;
  }
  if (hdr.len != VIDEO_PAYLOAD_MAX || (size_t)hdr.len + VIDEO_HDR_LEN != len) {
    return false;
  }
  memcpy(pix, payload + VIDEO_HDR_LEN, hdr.len);
  return true;
}

// Paint every complete chunk of one read block inside a single SPI
// transaction. A startWrite/endWrite pair per 1280 B chunk would pay the
// CS/DC setup 120 times per frame for nothing; the block is whatever the CDC
// queue had ready.
void blitBlock(const uint8_t *rx, size_t n, FrameParser &parser, uint16_t *pix, bool &open) {
  uint8_t payload[FRAME_MAX_PAYLOAD];
  for (size_t i = 0; i < n; i++) {
    uint8_t type = 0;
    uint16_t len = 0;
    if (!parser.push(rx[i], type, payload, len)) {
      continue;
    }
    if (type != FRAME_VIDEO && type != FRAME_VIDEO_RLE) {
      continue;
    }
    VideoChunkHdr hdr = {};
    if (!chunkPixels(type, payload, len, hdr, pix)) {
      continue;
    }
    if (!open) {
      gDisplay->beginBurst();
      open = true;
    }
    gDisplay->chunkInner(hdr, pix, (int)hdr.len);
    gFrames++;
  }
}

void videoTaskEntry(void *) {
  // Static and 4-byte aligned: 1280 B of 16-bit words is too much to keep on
  // the task stack next to the parser's own FRAME_MAX_PAYLOAD buffer.
  alignas(4) static uint16_t pix[VIDEO_PAYLOAD_MAX / 2];
  uint8_t scratch[1024];
  FrameParser parser;
  bool open = false;

  // CDC init happens here so its RX interrupt lands on this core (see the
  // header). Grow the RX queue first, or HWCDC::begin creates its 256 B
  // default and a burst overflows it.
  Serial.setRxBufferSize(CDC_RX_BYTES);
  Serial.begin(CDC_BAUD);
  xSemaphoreGive(gSerialReady);

  // boot() owns Serial until videoTaskGo() (it reads the CAL6 tokens).
  while (!gGo) {
    vTaskDelay(1);
  }

  for (;;) {
    const size_t n = Serial.read(scratch, sizeof scratch);
    if (n == 0) {
      if (open) {
        gDisplay->endBurst();
        open = false;
      }
      // Nothing queued: yield the core (the CDC ISR and the idle task need
      // it) and let the panel fall back to its RGB cycle — tick self-gates on
      // VIDEO_IDLE_MS, so calling it every idle millisecond is free.
      vTaskDelay(1);
      gDisplay->tick(millis());
      continue;
    }
    gBytes += n;
    blitBlock(scratch, n, parser, pix, open);
    if (open) {
      gDisplay->endBurst();
      open = false;
    }
    gDisplay->tick(millis());
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
