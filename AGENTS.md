# 6-DoF motion tracking puck

Two-part system: `device/` (ESP32-S3 firmware) streams IMU orientation over serial; `engine/` (tracked in root repo) reads it and renders OpenGL.

## Commands

- `make` — arduino-cli compile+upload `device`, then build+run engine
- `make monitor` — serial monitor for device
- `make compile-db` — generate `build/compile_commands.json` (needed before clangd works; `.clangd` reads it and strips ESP32 flags)
- `make engine` — build+run engine only
- `cd engine && make` / `make run` / `make clean` — engine (C++17, GLFW/GLAD)

## Architecture

- `device/device.ino`: MPU6050 DMP (MotionApps612) on I2C pins 4/5 — `Wire.begin(5, 4)` = SDA pin 5, SCL pin 4. Boot gyro calibration (100 samples, fresh each boot, outlier-guarded), prints `qw qx qy qz ax ay az temp` floats (quat wxyz, then world-frame linear accel m/s², gravity removed device-side), 115200 baud
- `engine/src/app.cpp` `getDeviceData()`: opens `/dev/ttyACM0` directly (hardcoded, O_NONBLOCK), parses 8 floats, normalizes quat, converts to Euler degrees for cube rotation; cube motion = accel → damped velocity (DAMPING_RATE/MAX_CUBE_SPEED/CUBE_BOUND in app.h), R resets to spawn
- Board config duplicated: `sketch.yaml` and `.arduino_config.lua` (esp32s3, /dev/ttyACM0)

## Gotchas

- No IMU attached → engine logs "failed to open /dev/ttyACM0" but continues running (identity quaternion)
- DMP settles ~10s after boot: quaternion sweeps from identity to true orientation, then holds (residual ~0.03°/s). Don't mistake the startup sweep for drift. Accel stream is garbage during the sweep too (gravity subtraction misaligned)
- Acceleration is gravity-free world-frame, but includes ~0.05 m/s² rest noise; engine has deadband (ACCEL_DEADBAND) — raise it if the cube crawls at rest
- Must run `make compile-db` after device code changes or clangd shows stale errors