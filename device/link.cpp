#include "link.h"
#include "packet.h"
#include "frame.h"

bool UsbLink::begin(unsigned long baud, size_t rxBuf) {
  Serial.setRxBufferSize(rxBuf);
  Serial.begin(baud);
  return true;
}

void UsbLink::sendQuat(const float quat[4], const float gyro[3]) {
  const bool doSend = sendNow_;
  sendNow_ = !sendNow_;
  if (!doSend) {
    return;
  }
  uint8_t payload[QUAT_PACKET_LEN_V2];
  build_quat_packet(payload, pktSeq_, micros(), quat, gyro);
  uint8_t frame[FRAME_HDR_LEN + QUAT_PACKET_LEN_V2 + FRAME_CRC_LEN];
  size_t n = build_frame(frame, FRAME_POSE, payload, QUAT_PACKET_LEN_V2);
  // Drop-if-backpressured: if the host is not draining (e.g. a video burst
  // is arriving while the reader stalls), skip this tick rather than stall
  // the 500 Hz fusion loop inside the CDC TX path.
  if (Serial.availableForWrite() < (int)n) {
    return;
  }
  Serial.write(frame, n);
  pktSeq_++;
}
