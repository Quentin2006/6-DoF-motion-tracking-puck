# 6-DoF Motion Tracking Puck

A self-contained head/puck tracker: an **ESP32-S3** puck reads an **MPU6050**
IMU, fuses accelerometer + gyroscope into a 6-DoF orientation quaternion with a
6-state EKF, and streams framed pose packets (~250 Hz — half `FUSION_RATE_HZ`,
because `BulkLink::sendQuat` alternates send/skip every tick) over a native USB
TinyUSB vendor-bulk link to a **Rust/Bevy** desktop engine. The engine drives a
look-around camera rig. The CH343 COM/UART bridge remains separate for flashing,
logs, and calibration.

The current prototype tracks orientation and renders a stereo view on two
320x240 panels. The camera uses rotation only; this project does not currently
provide positional tracking.

## Layout

- `device/` — ESP32-S3 Arduino firmware (C++, `arduino-cli`).
- `engine/` — Rust **Bevy 0.19.1** visualizer (`cargo`).
- `tests/` — cross-contract tests and the golden packet fixture (`make test`).

## Requirements

- `arduino-cli` with the ESP32 core (`arduino-cli core install esp32:esp32`).
- Arduino libraries `Adafruit GFX Library`, `Adafruit ST7735 and ST7789 Library`
  and `JPEGDEC` (`arduino-cli lib install ...`; the same three CI installs).
- Rust stable/Cargo and libusb development support (`libusb-1.0-0-dev` on
  Debian/Ubuntu).
- `jq` (used by `make lsp`) and `python3` (used by `make wire-check`).

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
make clean     # arduino-cli clean build
make lsp       # regenerate compile_commands.json for clangd
make wire-check # fail if the generated wire constants drifted from their spec
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

The 40-fps figure is a prior hardware measurement documented in the
architecture notes, not an automated benchmark result or a guarantee for
every scene. Contract tests validate wire bytes, framing, and fragment handling;
they do not measure sensor accuracy, end-to-end latency, or headset comfort.

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

## Learn the codebase

24 questions over the real code and the real measured numbers — what the SPI
throughput is and why it bounds everything, why rectangles instead of full
JPEGs, the comments that used to describe a protocol that never shipped, and
the exact byte layout of a frame, a pose payload, and a rectangle table. Every
answer cites the file and line it came from.

```sh
make quiz-open   # or just open quiz/index.html
```

One self-contained file, no build step, no network. Progress is kept in the
browser's localStorage.

See [AGENTS.md](AGENTS.md) for detailed architecture, conventions, and the
device↔engine wire contract.
