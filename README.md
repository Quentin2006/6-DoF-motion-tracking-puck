# 6-DoF Motion Tracking Puck

A self-contained head/puck tracker: an **ESP32-S3** puck reads an **MPU6050**
IMU, fuses accelerometer + gyroscope into a 6-DoF orientation quaternion with a
6-state EKF, and streams framed pose packets (~250 Hz) over a native USB
TinyUSB vendor-bulk link to a **Rust/Bevy** desktop engine. The engine drives a
look-around camera rig. The CH343 COM/UART bridge remains separate for flashing,
logs, and calibration.

## Layout

- `device/` — ESP32-S3 Arduino firmware (C++, `arduino-cli`).
- `engine/` — Rust **Bevy 0.19.1** visualizer (`cargo`).
- `tests/` — cross-contract tests and the golden packet fixture (`make test`).

## Requirements

- `arduino-cli` with the ESP32 core (`arduino-cli core install esp32:esp32`).
- Rust stable/Cargo and libusb development support (`libusb-1.0-0-dev` on
  Debian/Ubuntu).

## Build / run

One-time host setup for the native `303a:1002` device:

```sh
make usb-permissions
# unplug and replug the puck's native USB cable
```

Flash and use the two USB paths independently:

```sh
make compile   # build with USB-OTG TinyUSB; no auto-CDC interface
make upload    # flash over the CH343 COM/UART bridge ($(PORT))
make monitor   # COM/UART logs and CAL6 calibration
make engine    # run Bevy over native USB vendor bulk
make device    # compile + CH343 upload + CH343 monitor
make all       # compile + CH343 upload + bulk engine
make test      # Rust tests + C++ packet/frame/video contract tests
```

The FQBN is `esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=default,UploadMode=default`.
`make upload` and `make monitor` prefer the CH343 by-id node
(`/dev/serial/by-id/usb-1a86_USB_Single_Serial_*-if00`), fall back to
`/dev/ttyACM0`, and accept `PORT=<node>`. The engine does not use a serial-port
node: it claims the native USB device by VID:PID `303a:1002` and dynamically
selects its one class-`0xff` bulk IN/OUT interface.

Measure the raw host-to-puck bulk ceiling without JPEG encoding or panel work:

```sh
cd engine
cargo run -- --bulk-benchmark
```

The benchmark writes one reusable 64-KiB zero buffer for 10 seconds. Its
per-second and final lines report sent bytes, errors, and the observed rate;
any failed bulk write exits non-zero. It measures available transport
headroom, not a new panel-frame-rate guarantee: decode plus SPI blit remains the
measured 40-fps painted limit.

## Controls

The puck is the controller: its orientation *is* the rig's rotation, so the view
follows the puck directly. Keys only reach the engine while the visualizer
window has keyboard focus.

- `R` — recenter: the orientation the puck is in right now becomes the reference
  view (the rig ends up looking down -Z at the reference object). Full
  orientation, all three axes, and it lands in one frame rather than easing.

## Status LED

- **red** — booting / calibrating / not ready
- **green** — calibrated, streaming pose over USB vendor bulk

See [AGENTS.md](AGENTS.md) for detailed architecture, conventions, and the
device↔engine wire contract.
