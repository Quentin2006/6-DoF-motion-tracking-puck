# 6-DoF motion tracking puck

Two-part system: `device/` (ESP32-S3 firmware) streams IMU orientation over serial; `engine/` (tracked in root repo) reads it and renders OpenGL.

## Commands

- `make` — arduino-cli compile+upload `device`, then build+run engine
- `make monitor` — serial monitor for device
- `make compile-db` — generate `build/compile_commands.json` (needed before clangd works; `.clangd` reads it and strips ESP32 flags)
- `make engine` — build+run engine only
- `cd engine && make` / `make run` / `make clean` — engine (C++17, GLFW/GLAD)

## Architecture

- `device/device.ino`: MPU6050 DMP on I2C pins 4/5, boot gyro calibration, prints `qw qx qy qz temp` floats, 115200 baud
- `engine/src/app.cpp` `getDeviceData()`: opens `/dev/ttyACM0` directly (hardcoded, O_NONBLOCK), parses 5 floats (quaternion wxyz), normalizes, converts to Euler degrees for cube rotation
- Board config duplicated: `sketch.yaml` and `.arduino_config.lua` (esp32s3, /dev/ttyACM0)

## Gotchas

- No IMU attached → engine logs "failed to open /dev/ttyACM0" but continues running (identity quaternion)
- Must run `make compile-db` after device code changes or clangd shows stale errors