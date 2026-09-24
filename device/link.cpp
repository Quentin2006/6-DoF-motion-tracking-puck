#include "link.h"
#include "packet.h"
#include "frame.h"
#include "src/transport/usb_bulk.h"

void BulkLink::sendQuat(const float quat[4], const float gyro[3]) {
  const bool doSend = sendNow_;
  sendNow_ = !sendNow_;
  if (!doSend) {
    return;
  }
  uint8_t payload[QUAT_PACKET_LEN_V2];
  build_quat_packet(payload, pktSeq_, micros(), quat, gyro);
  uint8_t frame[FRAME_HDR_LEN + QUAT_PACKET_LEN_V2 + FRAME_CRC_LEN];
  size_t n = build_frame(frame, FRAME_POSE, payload, QUAT_PACKET_LEN_V2);
  // Drop-if-backpressured: if the host is not draining, skip this tick rather
  // than stall the 500 Hz fusion loop. try_write() only accepts a complete frame.
  if (BulkUsb::write_available() < n) {
    (void)BulkUsb::try_write(frame, n); // account for the non-blocking drop
    return;
  }
  if (!BulkUsb::try_write(frame, n)) {
    return;
  }
  pktSeq_++;
}
