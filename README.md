# 6-DoF Motion Tracking Puck

A self-contained head/puck tracker: an **ESP32-S3** puck reads an **MPU6050**
IMU, fuses accelerometer + gyroscope into a 6-DoF orientation quaternion with a
6-state EKF, and streams the pose as framed packets (~250 Hz) over USB-CDC
serial to a **Rust/Bevy** desktop engine that drives a look-around camera rig.

## Layout

- `device/` — ESP32-S3 Arduino firmware (C++, `arduino-cli`).
- `engine/` — Rust **Bevy 0.19.1** visualizer (`cargo`).
- `tests/` — cross-contract test + golden packet fixture (`make test`).

## Requirements

- `arduino-cli` with the ESP32 core (`arduino-cli core install esp32:esp32`).

## Build / run

```sh
make compile   # build the firmware
make upload    # flash to /dev/ttyACM0
make monitor   # serial console
make device    # compile + upload + monitor
make engine    # run the Bevy visualizer
make all       # compile + upload + engine
make test      # engine unit tests + host packet-contract test
```
FQBN is `esp32:esp32:esp32s3:CDCOnBoot=cdc`, port `/dev/ttyACM0` at 921600
(`PUCK_SERIAL` env overrides the port).

## Status LED

- **red** — booting / calibrating / not ready
- **green** — calibrated, streaming pose over USB-CDC

See [AGENTS.md](AGENTS.md) for detailed architecture, conventions, and the
device↔engine wire contract.
