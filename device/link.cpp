#include "link.h"
#include "packet.h"
#include "frame.h"
#include <cstring>

bool UsbLink::begin(unsigned long baud, size_t rxBuf) {
  Serial.setRxBufferSize(rxBuf);
  Serial.begin(baud);
  return true;
}

int UsbLink::pollVideoChunk(uint8_t *out, VideoChunkHdr &hdr) {
  // One pump per call: move at most one segment out of the 4 KB FreeRTOS
  // queue behind the previous call's stashed tail, then run the frame state
  // machine over the combined input. The parser holds partial-frame state
  // across calls, so a back-to-back video burst spreads over many loop calls
  // instead of stalling one 500 Hz fusion tick — the panel blit absorbs the
  // delay, the pose stream never does. Bytes beyond the returned chunk wait
  // in stash_ for the next call, never in the void: the parser resets on
  // every completed frame, so the stashed tail is unconsumed input and
  // re-feeding it next call is exact. Single block read (not per-byte
  // Serial.read) so the video burst costs one FreeRTOS copy per loop, not
  // ~1300 virtual calls. inN <= sizeof in by construction below: the fresh
  // read is capped at one segment AND at the staging room left by the
  // stashed tail (stash_ is 2x a segment, so room can be under a segment),
  // and the stashed tail after a returned chunk (tail < inN <= sizeof
  // stash_) always fits — every input byte is either parsed or stashed,
  // except the defensive-only overflow branch, which counts the loss.
  constexpr size_t kSegMax = FRAME_MAX_PAYLOAD + 16;
  uint8_t in[2 * kSegMax];
  size_t inN = 0;
  if (stashN_ > 0) {
    memcpy(in, stash_, stashN_);
    inN = stashN_;
    stashN_ = 0;
  }
  const int avail = Serial.available();
  if (avail > 0 && inN < sizeof in) {
    size_t want = (size_t)avail;
    if (want > kSegMax) {
      want = kSegMax;
    }
    const size_t room = sizeof in - inN;
    if (want > room) {
      want = room;
    }
    const size_t got = Serial.readBytes((char *)(in + inN), want);
    rxBytes_ += (uint32_t)got;
    inN += got;
  }
  if (inN == 0) {
    return 0;
  }
  uint8_t payload[FRAME_MAX_PAYLOAD];
  for (size_t i = 0; i < inN; i++) {
    uint8_t type = 0;
    uint16_t len = 0;
    if (!parser_.push(in[i], type, payload, len)) {
      continue;
    }
    if (type != FRAME_VIDEO && type != FRAME_VIDEO_RLE) {
      // A completed non-video frame: fully parsed, but yields no chunk.
      // Count it so no consumed input goes unaccounted (host never sends
      // these; a climb here means link crosstalk, not video loss).
      decodeDrops_++;
      continue;
    }
    if (!parse_video_header(payload, len, hdr) || hdr.len == 0 ||
        (size_t)hdr.len + VIDEO_HDR_LEN > len) {
      decodeDrops_++;
      continue;
    }
    if (type == FRAME_VIDEO) {
      // Raw path: hdr.len must be the full chunk (Display::chunkInner blits
      // a fixed 2-row window; hdr.len == VIDEO_PAYLOAD_MAX per the link.h
      if (hdr.len != VIDEO_PAYLOAD_MAX || (size_t)hdr.len + VIDEO_HDR_LEN != len) {
        decodeDrops_++;
        continue;
      }
      memcpy(out, payload + VIDEO_HDR_LEN, hdr.len);
    } else {
      // RLE path: decode into the caller's aligned buffer (out is 16-bit
      // aligned per the link.h contract); decode failure drops the chunk.
      if (!rle_decode_565(payload + VIDEO_HDR_LEN, hdr.len, (uint16_t *)out)) {
        decodeDrops_++;
        continue;
      }
      hdr.len = VIDEO_PAYLOAD_MAX;
    }
    chunks_++;
    // Stash (never silently drop) the input after this chunk: it belongs to
    // later frames and paints on the next calls, so every received row
    // eventually blits. tail < inN <= sizeof stash_ by construction, so the
    // fit is exact; the overflow branch below is defensive-only (its loss is
    // still counted, never silent).
    const size_t tail = inN - (i + 1);
    if (tail > sizeof stash_) {
      decodeDrops_++;
      memcpy(stash_, in + inN - sizeof stash_, sizeof stash_);
      stashN_ = sizeof stash_;
    } else if (tail > 0) {
      memcpy(stash_, in + (i + 1), tail);
      stashN_ = tail;
    }
    return (int)hdr.len;
  }
  // No complete chunk this call: every inN byte ran through the parser, so
  // partial-frame state carries over and nothing is lost or needs stashing.
  // The only consumed-and-gone bytes are completed non-video frames or
  // decode/header rejects, each counted at its continue site above.
  return 0;
}

bool UsbLink::sendQuat(const float quat[4], const float gyro[3]) {
  const bool doSend = sendNow_;
  sendNow_ = !sendNow_;
  if (!doSend) {
    return true;
  }
  uint8_t payload[QUAT_PACKET_LEN_V2];
  build_quat_packet(payload, pktSeq_, micros(), quat, gyro);
  uint8_t frame[FRAME_HDR_LEN + QUAT_PACKET_LEN_V2 + FRAME_CRC_LEN];
  size_t n = build_frame(frame, FRAME_POSE, payload, QUAT_PACKET_LEN_V2);
  // Drop-if-backpressured: if the host is not draining (e.g. a video burst
  // is arriving while the reader stalls), skip this tick rather than stall
  // the 500 Hz fusion loop inside TinyUSB's blocking TX path.
  if (Serial.availableForWrite() < (int)n) {
    dropped_++;
    return false;
  }
  Serial.write(frame, n);
  pktSeq_++;
  return true;
}
