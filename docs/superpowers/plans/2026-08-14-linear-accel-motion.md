# Linear Acceleration Motion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stream gravity-free world-frame linear acceleration from the puck; engine integrates it into cube velocity with damping, clamping, scene bounds, and an R-key reset.

**Architecture:** DMP FIFO packet already carries raw accel — device reads it alongside the quaternion, rotates sensor→world via the quaternion, subtracts gravity, and appends 3 floats to serial (`qw qx qy qz ax ay az temp`, 8 floats). Engine parses 8 floats, integrates accel→velocity→position with exponential damping, and resets via new R key binding.

**Tech Stack:** MPU6050 lib 1.4.5 (MotionApps612), arduino-cli, C++17/GLFW engine.

---

### Task 1: Device — world-frame linear acceleration

**Files:** `device/device.ino`

- [ ] **Step 1:** Add rotation helper + accel output in `loop()`

```cpp
void rotateByQuat(const float qw, const float qx, const float qy, const float qz,
                  float &vx, float &vy, float &vz) {
  const float qvw = qw, qvx = qx, qvy = qy, qvz = qz;
  const float w2 = qw * qw - (qx * qx + qy * qy + qz * qz);
  const float dot = qx * vx + qy * vy + qz * vz;
  const float cx = qy * vz - qz * vy;
  const float cy = qz * vx - qx * vz;
  const float cz = qx * vy - qy * vx;
  vx = w2 * vx + 2.0f * dot * qx + 2.0f * qw * cx;
  vy = w2 * vy + 2.0f * dot * qy + 2.0f * qw * cy;
  vz = w2 * vz + 2.0f * dot * qz + 2.0f * qw * cz;
}

void loop() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    int16_t ax, ay, az;
    mpu.dmpGetAccel(&ax, &ay, &az, fifoBuffer);
    constexpr float ACCEL_SCALE = 9.81f / 16384.f;
    float wx = ax * ACCEL_SCALE, wy = ay * ACCEL_SCALE, wz = az * ACCEL_SCALE;
    rotateByQuat(q.w, q.x, q.y, q.z, wx, wy, wz);
    wx -= 0.0f; wy -= 0.0f; wz -= 9.81f;  // world +Z = up (verify, see notes)
    float temp = mpu.getTemperature() / 340.0f + 36.53f;
    Serial.printf("%.4f %.4f %.4f %.4f %.3f %.3f %.3f %.2f\n",
                  q.w, q.x, q.y, q.z, wx, wy, wz, temp);
  }
}
```

- [ ] **Step 2:** Compile: `arduino-cli compile device` — success
- [ ] **Step 3:** Upload + verify with pyserial (DTR/RTS reset pulse):
  - At rest, puck flat: `ax ay az` ≈ 0,0,0 (±0.5)
  - Rotate puck: accel changes opposite gravity direction
  - Flick upward: +Z spike (if spike appears on −Z, swap the gravity subtraction to `+9.81` in the `wz` line — convention note)
  - Rotate while moving: |accel| > 0 during motion
- [ ] **Step 4:** Commit: `git add device/device.ino && git commit -m "feat: stream world-frame linear acceleration"`

**Notes:** `dmpGetAccel` (MotionApps612.h:85) returns the same-packet raw accel — no extra I2C read. Verify gravity sign empirically per Step 3; if gravity lands on wrong axis, flip `wz` sign — one-line fix.

### Task 2: Engine — parse + physics

**Files:** `engine/src/app.cpp`, `engine/src/app.h`, `engine/src/controls.h`, `engine/src/callbacks.cpp`

- [ ] **Step 1:** `app.cpp` DeviceData — add field:

```cpp
struct DeviceData {
  glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};  // wxyz from DMP
  glm::vec3 accel{0.0f};                            // m/s², world frame
  float temperature = 0;                            // °C
};
```

- [ ] **Step 2:** `app.cpp` parse loop — 8 floats:

```cpp
    glm::quat quat;
    glm::vec3 accel;
    float temp;
    if (line >> quat.w >> quat.x >> quat.y >> quat.z >> accel.x >>
        accel.y >> accel.z >> temp) {
      last.orientation = glm::normalize(quat);
      last.accel = accel;
      last.temperature = temp;
    }
```

- [ ] **Step 3:** `app.h` — physics state + input:

```cpp
struct InputState {
  // ...existing...
  bool r = false;
  bool r_pressed = false;
};
// in App private:
glm::vec3 cubeVelocity{0.0f};
glm::vec3 cubeSpawn{0.0f};
```

- [ ] **Step 4:** `controls.h` — add `static const int RESET = GLFW_KEY_R;`
- [ ] **Step 5:** `callbacks.cpp` — add `case Controls::RESET: input->r = pressed; return;`
- [ ] **Step 6:** `app.cpp` run loop — physics after `cube.rotation` line:

```cpp
    // velocity + damping + clamp + bounds
    cubeVelocity += deviceData.accel * deltaTime;
    cubeVelocity *= std::exp(-2.5f * deltaTime);
    if (glm::length(cubeVelocity) > 12.0f)
      cubeVelocity = glm::normalize(cubeVelocity) * 12.0f;
    cube.position += cubeVelocity * deltaTime;
    constexpr float BOUND = 12.0f;
    if (glm::length(cube.position) > BOUND) {
      cube.position = glm::normalize(cube.position) * BOUND;
      cubeVelocity = glm::vec3(0.0f);
    }
    if (input.r && !input.r_pressed) {
      input.r_pressed = true;
      cube.position = cubeSpawn;
      cubeVelocity = glm::vec3(0.0f);
    } else if (!input.r) {
      input.r_pressed = false;
    }
```

(capture `cubeSpawn = cube.position` once before the loop; include `<cmath>` if missing)

- [ ] **Step 7:** Build: `cd engine && make` — clean (pre-existing `totalTime` warning OK)
- [ ] **Step 8:** Run with puck: `make engine` — tilting/throwing puck moves cube; damping settles it; R resets; bounds clamp
- [ ] **Step 9:** Commit: `git add engine/src && git commit -m "feat: integrate linear acceleration into cube motion (damped velocity)"`

### Task 3: Docs

**Files:** `AGENTS.md`

- [ ] **Step 1:** Update protocol line: 8 floats `qw qx qy qz ax ay az temp` (accel m/s² world-frame)
- [ ] **Step 2:** Add gotcha: accel = linear (gravity removed, device-side); startup sweep same as quat
- [ ] **Step 3:** Commit: `git add AGENTS.md && git commit -m "docs: document linear acceleration protocol"`

---

**Open conventions (verified empirically, one-line fixes):**
- Gravity sign: world +Z = up assumed; flip `wz -=` if spike appears inverted
- Scene up-axis: if cube flies sideways instead of up, map `(ax, ay, az)` → scene axes in one line (likely `(ax, az, ay)` swap — verify during Task 2 Step 8)

**Honest limits:** velocity is integrated, so long holds accumulate drift; damping + bounds + R reset contain it. No position lock — that needs external reference hardware (UWB/vision), out of scope.