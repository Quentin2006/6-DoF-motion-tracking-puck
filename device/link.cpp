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
  // queue, then run the frame state machine over it. The parser holds
  // partial-frame state across calls, so a back-to-back video burst spreads
  // over many loop calls instead of stalling one 500 Hz fusion tick — the
  // panel blit absorbs the delay, the pose stream never does. Bytes beyond
  // the segment wait in the queue for the next calls. Single block read
  // (not per-byte Serial.read) so the ~10 KB/s video burst costs one
  // FreeRTOS copy per loop, not ~1300 virtual calls.
  uint8_t rxBuf[FRAME_MAX_PAYLOAD + 16];
  const size_t want = sizeof rxBuf;
  const int avail = Serial.available();
  const size_t rxN = (avail <= 0) ? 0 : Serial.readBytes((char *)rxBuf, (size_t)(avail < (int)want ? avail : (int)want));
  rxBytes_ += (uint32_t)rxN;
  if (rxN == 0) {
    return 0;
  }
  uint8_t payload[FRAME_MAX_PAYLOAD];
  for (size_t i = 0; i < rxN; i++) {
    uint8_t type = 0;
    uint16_t len = 0;
    if (!parser_.push(rxBuf[i], type, payload, len)) {
      continue;
    }
    if (type != FRAME_VIDEO && type != FRAME_VIDEO_RLE) {
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
    return (int)hdr.len;
  }
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
