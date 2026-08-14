# DMP Orientation Tracking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace raw MPU6050 streaming with on-chip DMP quaternion output, and drive engine cube rotation from it.

**Architecture:** `device.ino` boots with gyro bias calibration, enables DMP, streams `qw qx qy qz temp` (ASCII, 115200). `app.cpp` parses 5 floats into `DeviceData`, converts quaternion to Euler degrees for existing `Transform.rotation` pipeline.

**Tech Stack:** MPU6050 lib 1.4.5 (installed, `~/Arduino/libraries/MPU6050`, MotionApps20 DMP firmware included), arduino-cli, C++17/GLFW engine.

---

### Task 1: Rewrite device.ino with DMP + boot calibration

**Files:**
- Modify: `device/device.ino` (full rewrite)

- [ ] **Step 1: Replace file contents**

```cpp
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
Quaternion q;
uint16_t packetSize;
uint8_t fifoBuffer[64];

void calibrateGyro() {
  const int SAMPLES = 200;
  int32_t sx = 0, sy = 0, sz = 0;
  int16_t x, y, z;
  for (int i = 0; i < SAMPLES; i++) {
    mpu.getRotation(&x, &y, &z);
    sx += x; sy += y; sz += z;
    delay(5);
  }
  mpu.setXGyroOffset(-sx / SAMPLES);
  mpu.setYGyroOffset(-sy / SAMPLES);
  mpu.setZGyroOffset(-sz / SAMPLES);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  Wire.begin(4, 5);

  Serial.println("Initing MPU6050...");
  mpu.initialize();
  calibrateGyro();

  uint8_t err = mpu.dmpInitialize();
  if (err != 0) {
    Serial.print("DMP init failed: ");
    Serial.println(err);
    return;
  }
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  Serial.println("DMP ready");
}

void loop() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    float temp = 0;
    mpu.dmpGetTemperature(&temp, fifoBuffer);
    Serial.printf("%.4f %.4f %.4f %.4f %.2f\n", q.w, q.x, q.y, q.z, temp);
  }
}
```

Notes: `dmpGetCurrentFIFOPacket` is overflow-proof (resets FIFO). Offsets must be set before `dmpInitialize()` — it won't pick them up after. Removed unused `<WiFi.h>`.

- [ ] **Step 2: Compile**

Run: `arduino-cli compile device`
Expected: success, no warnings

- [ ] **Step 3: Upload + verify stream**

Run: `arduino-cli upload device && arduino-cli monitor device`
Expected: "DMP ready", then lines like `0.9998 -0.0042 0.0131 0.0050 27.84`; quat changes when board rotated; board must be stationary ~1s during boot (gyro cal)

- [ ] **Step 4: Commit (root repo only)**

```bash
git add device/device.ino
git commit -m "feat: use MPU6050 DMP quaternion output with boot gyro calibration"
```

### Task 2: Engine — parse quaternion stream

**Files:**
- Modify: `engine/src/app.cpp:111-167`

- [ ] **Step 1: Replace DeviceData struct + parser**

`engine/src/app.cpp:111` — replace:

```cpp
struct DeviceData {
  glm::vec3 accel;        // m/s²
  glm::vec3 gyro;         // °/s
  float temperature = 0;  // °C
};
```

with:

```cpp
struct DeviceData {
  glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};  // wxyz from DMP
  float temperature = 0;                            // °C
};
```

`engine/src/app.cpp:155-164` — replace the parse loop body:

```cpp
    std::array<int, 6> raw{};
    if (std::all_of(raw.begin(), raw.end(),
                    [&line](int &v) { return static_cast<bool>(line >> v); })) {
      constexpr float ACCEL_SCALE = 9.81f / 16384.f;
      constexpr float GYRO_SCALE = 1.f / 131.f;
      last.accel = {raw[0] * ACCEL_SCALE, raw[1] * ACCEL_SCALE,
                    raw[2] * ACCEL_SCALE};
      last.gyro = {raw[3] * GYRO_SCALE, raw[4] * GYRO_SCALE,
                   raw[5] * GYRO_SCALE};
      line >> last.temperature;
    }
```

with:

```cpp
    glm::quat quat;
    float temp;
    if (line >> quat.w >> quat.x >> quat.y >> quat.z >> temp) {
      last.orientation = quat;
      last.temperature = temp;
    }
```

(`glm::quat` members are public; `>>` on floats works. Keeps the `last`-value fallback behavior when no data.)

- [ ] **Step 2: Update cube orientation usage**

`engine/src/app.cpp:225-227` — replace:

```cpp
    auto &cube = registry.getTransform(0);
    cube.position = deviceData.accel;
    cube.rotation = deviceData.gyro;
```

with:

```cpp
    auto &cube = registry.getTransform(0);
    cube.rotation = glm::degrees(glm::eulerAngles(deviceData.orientation));
```

(`Transform.rotation` is Euler degrees; `systems.h:23-27` applies Rx→Ry→Rz.)

- [ ] **Step 3: Build engine**

Run: `cd engine && make`
Expected: compiles clean

- [ ] **Step 4: Verify on hardware**

Run: `make` (device upload + engine)
Expected: cube rotates matching physical puck orientation. Caveat: if rotation axes appear permuted (eulerAngles convention vs RxRyRz order), swap/negate axes in the `eulerAngles` line — noted as known tuning point, not a blocker.

- [ ] **Step 5: Commit (engine repo)**

```bash
git -C engine add src/app.cpp
git -C engine commit -m "feat: parse DMP quaternion stream and drive cube orientation"
```

### Task 3: Update docs

**Files:**
- Modify: `AGENTS.md`

- [ ] **Step 1: Update architecture + gotchas**

Replace `device/device.ino` line: MPU6050 on I2C pins 4/5 → DMP enabled, boot gyro calibration, prints `qw qx qy qz temp` floats.

Replace `getDeviceData()` line: opens `/dev/ttyACM0`, parses 5 floats, quaternion wxyz.

- [ ] **Step 2: Commit (root repo)**

```bash
git add AGENTS.md
git commit -m "docs: update AGENTS.md for DMP quaternion protocol"
```

---

**Notes:** No unit test framework exists in engine — verification is hardware-based (`make monitor` / `make run`). Device side has no test story; compile + serial observation is the check. `make compile-db` only matters for clangd, not builds.