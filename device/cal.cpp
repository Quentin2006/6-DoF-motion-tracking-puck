#include "cal.h"
#include "config.h"

// Wait up to timeoutMs for a serial line; trims CR/LF. Returns true on line.
bool AccelCalibrator::waitLine(String &line, unsigned long timeoutMs) {
  unsigned long t0 = millis();
  line = "";
  while ((unsigned long)(millis() - t0) < timeoutMs) {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        if (line.length() > 0) return true;
        // else keep waiting for content within the window
      } else {
        line += c;
      }
    }
    delay(10);
  }
  line.trim();
  return line.length() > 0;
}

bool AccelCalibrator::awaitTrigger(unsigned long timeoutMs) {
  String cmd;
  return waitLine(cmd, timeoutMs) && cmd == "CAL6";
}

// One-time serial-guided 6-point accel cal. Blocking, setup-time only.
// Any unexpected input or timeout aborts WITHOUT saving (NVS untouched).
void AccelCalibrator::run(Mpu6050 &mpu) {
  static const char *kPoseName[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
  vec3 means[6];
  for (int p = 0; p < 6; ++p) {
    Serial.print("CAL6 pose ");
    Serial.print(kPoseName[p]);
    Serial.println(" down: hold still, send G");
    String cmd;
    if (!waitLine(cmd, 120000UL) || cmd != "G") {
      Serial.println("CAL6 aborted, nothing saved");
      return;
    }
    // ~5 s raw mean, bypassing all driver-side accel calibration.
    vec3 total;
    unsigned long n = 0;
    unsigned long t0 = millis();
    while ((unsigned long)(millis() - t0) < 5000UL) {
      vec3 s;
      if (mpu.readRawAccel(s)) {
        total += s;
        ++n;
      }
      delay(5);
    }
    if (n == 0) {
      Serial.println("CAL6 aborted, nothing saved (no samples)");
      return;
    }
    total /= (float)n;
    means[p] = total;
    Serial.print("CAL6 mean ");
    Serial.print(kPoseName[p]);
    Serial.print(' ');
    Serial.print(total.x, 4);
    Serial.print(' ');
    Serial.print(total.y, 4);
    Serial.print(' ');
    Serial.println(total.z, 4);
  }
  float off[3], scale[3];
  for (int i = 0; i < 3; ++i) {
    const vec3 &pa = means[2 * i];
    const vec3 &pb = means[2 * i + 1];
    float a = i == 0 ? pa.x : (i == 1 ? pa.y : pa.z);
    float b = i == 0 ? pb.x : (i == 1 ? pb.y : pb.z);
    float mx = a > b ? a : b;
    float mn = a < b ? a : b;
    if ((mx - mn) < 1e-6f) {
      Serial.println("CAL6 aborted, nothing saved (degenerate span)");
      return;
    }
    off[i] = (mx + mn) * 0.5f;
    scale[i] = 2.0f * GRAVITY_MPS2 / (mx - mn);
  }
  Serial.print("CAL6 solved off ");
  Serial.print(off[0], 4);
  Serial.print(' ');
  Serial.print(off[1], 4);
  Serial.print(' ');
  Serial.print(off[2], 4);
  Serial.print(" scale ");
  Serial.print(scale[0], 4);
  Serial.print(' ');
  Serial.print(scale[1], 4);
  Serial.print(' ');
  Serial.println(scale[2], 4);
  Serial.println("CAL6 send SAVE to flash, anything else aborts");
  String save;
  if (!waitLine(save, 120000UL) || save != "SAVE") {
    Serial.println("CAL6 aborted, nothing saved");
    return;
  }
  mpu.setAccelCal(off, scale);
  mpu.getAccelBias(GYRO_CALIB_SAMPLES); // re-seed runtime offset residual
  Serial.println("CAL6 saved; reboot");
}
