# Repository Guidelines

## Project Overview

A 6-DoF motion-tracking puck. An ESP32-S3 puck reads an MPU6050 IMU, fuses
accelerometer + gyroscope into a 6-DoF orientation quaternion with a 6-state
EKF, and streams the pose as framed CDC-serial packets (~250 Hz) to a
Rust/Bevy desktop engine. The engine opens the puck serial port, parses the
frames, and drives a **look-around camera rig** — the puck's orientation maps
directly to camera rotation, so the user moves the puck and the 3D view
follows.

Two independent halves, no shared code, wired together by a single root
`Makefile` and hand-maintained constant agreement:

- `device/` — ESP32-S3 Arduino firmware (C++ / `.ino` + `.cpp`, `arduino-cli`).
- `engine/` — Rust **Bevy 0.19.1** desktop visualizer (`cargo run`).

## Architecture & Data Flow

```
 MPU6050 (I2C SDA=5 SCL=4 @400kHz)
   │  readGyro() dps + readAccel() m/s²   (mpu6050.cpp: cal offset/scale + bias)
   ▼
 Tracker::step()  (src/position/tracker.cpp)
   │  toBody swizzle → body frame; body-accel norm
   ├─ StillDetector::update(driver-frame gyro + norm) → Mpu6050.nudgeGyroBias (clamped 0.002 dps/tick)
   ├─ Ekf6Dof::predict(gyro rad/s) → Ekf6Dof::updateAccel(accel)  [gated → gated_++]
   └─ wrap to display quat  (fixed ±90° X sandwich, right-multiply)
   ▼
 device.ino loop() → UsbLink::sendQuat(quat, gyro)
   │  FRAME_POSE frame: magic 0x55 0xAA | type u8 | len u16 LE | 36 B payload | crc u16 LE
   │  payload: seq u32 | stamp u32 | quat xyzw f32 | gyro xyz f32  (all LE)
   ▼
 engine/src/tracking.rs::update  (serial: native USB 303a:1001 @921600, drain-to-latest)
   ▼
 LookRig camera Transform.rotation
```

Video runs the other way down the same port, at its own pace:

```
 Bevy 320x240 RGBA render target → readback → jpeg-encoder q75 4:2:0 (~8-15 kB/frame)
   │  FRAME_VIDEO fragment: 8 B header (frame/frag/count/len) + <= 1280 B JPEG
   │  one fragment per slice of the 33 ms frame (never a burst: the USJ RX FIFO is 64 B)
   ▼
 device/src/display/video_task.cpp  (core-0 task)
   │  VideoFragAssembler → JPEGDEC → Display::blitBlock per MCU block
   ▼
 ST7789 panel @ ~30 fps
```

**Key modules (`device/`)**

| File | Role |
|---|---|
| `device.ino` | Orchestrator: `setup()` `videoTaskStart()` (core-0 CDC init) → init/calib → `videoTaskGo()`, `loop()` (tick → send → report → video counters) |
| `config.h` | Single source of all compile-time constants (pins, FSR, gravity, CDC framing, all EKF sigmas/gates, axis map, display pins) |
| `src/position/tracker.{h,cpp}` | Pipeline glue + `micros()` scheduler + CAL6 accelerometer calibration + 1 Hz status line; owns `Mpu6050`, `Ekf6Dof`, `StillDetector` |
| `src/position/mpu6050.{h,cpp}` | I2C driver + NVS accel-cal persistence (`Preferences`, namespace `mpu`, keys `ax0-2`/`as0-2`) |
| `src/position/fusion.{h,cpp}` | `Ekf6Dof` — 6-state EKF (quat w,x,y,z + gyro bias) |
| `src/position/still_detect.{h,cpp}` | `StillDetector` — gyro re-bias while stationary (50-tick window, variance/norm gates) |
| `link.{h,cpp}` | USB-CDC pose-out link (`Serial`): FRAME_POSE frames, sequence counter |
| `frame.h` | Framed CDC byte-stream contract (magic/type/len/CRC) — host-compilable, parsed by `link.cpp`/`video_task.cpp` and `engine/src/tracking.rs` |
| `src/display/display.{h,cpp}` | ST7789 panel: MCU-block blit, burst transaction, idle RGB cycle |
| `src/display/video_task.{h,cpp}` | Core-0 task owning the whole Serial path: CDC init (so its RX interrupt runs on core 0), JPEG fragment reassembly, JPEGDEC decode + panel blit, 1 Hz `vid/drop/bad/kb` counters |
| `src/display/video.h` | Video wire contract: JPEG fragment header + `VideoFragAssembler` — host-compilable, mirrored by `engine/src/tracking.rs` |
| `packet.h` | Canonical 36 B pose payload + `build_quat_packet()` — host-compilable, single source of the byte contract |

**Key module (`engine/`)**

| File | Role |
|---|---|
| `src/main.rs` | `App::new().add_plugins(DefaultPlugins).add_plugins(TrackPlugin).run()` |
| `src/tracking.rs` | All behavior: `SerialLink`/`VideoTx` resources, `LookRig` marker, `TrackPlugin`, `update` ECS system, serial reader + MJPEG encode/fragment/paced sender threads |

**Cross-file wire contract (edit both halves together).** `device/frame.h`,
`device/packet.h`, `device/src/display/video.h` and `engine/src/tracking.rs`
hard-code the same framed-CDC protocol by hand; there is no shared crate or
serializer. The 36-byte pose payload (`QUAT_PACKET_LEN_V2`) inside a
`FRAME_POSE` frame, and the 320×240 baseline-JPEG video fragments (`VIDEO_*`),
**must match in both halves**. Both x86 and ESP32 are little-endian, so no
byte-swapping is done.

The fragment layout is **canonical in `device/src/display/video.h`**
(`VideoFragAssembler`, host-compilable, no Arduino deps; the engine's
`send_jpeg` is the producer). `tests/video_test.cpp` and
`tracking.rs::video_fragment_wire_layout_is_pinned` assert the same literal
bytes for one fragment, so a one-sided layout change fails `make test`.

The packet byte layout is **canonical in `device/packet.h`**
(`build_quat_packet`, host-compilable, no Arduino deps); `link.cpp`
(`UsbLink::sendQuat`) calls it and names `QUAT_PACKET_LEN_V2` directly. The committed `tests/packet_fixture.txt`
is the single source of truth and is asserted by **both** halves — the C++ host
test (`tests/packet_test.cpp`, producer) and the Rust parser test
(`tracking.rs`, consumer). Changing the packet layout means updating
`device/packet.h`, `tests/packet_fixture.txt`, and the parser — any one-sided
edit fails `make test`.

## Key Directories

- `device/` — ESP32-S3 Arduino sketch (source of truth for firmware).
- `engine/` — Rust Bevy visualizer (`src/main.rs`, `src/tracking.rs`).
- `tests/` — cross-contract tests: `packet_fixture.txt` (shared golden bytes), `packet_test.cpp` (C++ producer vs fixture), `frame_test.cpp` (framing/CRC/desync), `video_test.cpp` (JPEG fragment contract, ASAN).
- `.github/` — CI (`workflows/ci.yml`): engine `cargo test` + device `arduino-cli compile`.
- `build/` — arduino-cli output incl. raw `build/compile_commands.json` (gitignored).
- Root datasheets `ST7789VW.PDF` (display), `MPU-6000.PDF` (IMU) — reference only.
- No `docs/`, `scripts/`; all test infra lives in `tests/` + CI.

## Development Commands

Everything is driven from the root `Makefile`. Two real, authoritative build
chains: **`arduino-cli`** (device) and **`cargo`** (engine). `clangd`/`.clangd`
is editor IntelliSense only — it does not compile or flash.

```bash
make compile          # build firmware only: arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc device
make upload           # flash to $(PORT): arduino-cli upload --fqbn $(FQBN) -p $(PORT) device
make monitor          # serial monitor on $(PORT) @921600 (device/sketch.yaml baud)
make device           # compile + upload + monitor  (interactive loop)
make engine           # cd engine && cargo run      (host Bevy visualizer)
make all              # compile + upload + engine   (full pipeline)
make clean            # arduino-cli compile --clean --fqbn $(FQBN) device
make lsp              # regenerate + rewrite compile_commands.json for clangd
make test             # cargo test (engine) + host contract tests (packet, framing, video fragments)
```

- **FQBN** default `esp32:esp32:esp32s3:CDCOnBoot=cdc` (in `Makefile` `?=` and `device/sketch.yaml`); override with `make compile FQBN=<other>`.
- **Engine port:** the native-USB puck — selected by USB VID:PID `303a:1001` from `serialport::available_ports()`, because the `ttyACM<n>` index depends on plug order (bridge `1a86:55d3` + native both plugged ⇒ native is not `ttyACM0`). `PUCK_SERIAL` overrides with any explicit node. **Baud** 921600 (native-CDC baud is don't-care). Firmware flash/monitor uses `Makefile`'s `PORT`, which resolves the same way order-independently (`/dev/serial/by-id/usb-Espressif_USB_JTAG_*`) and falls back to the literal `/dev/ttyACM0` (the UART bridge, where a bridge-only setup enumerates); override with `make upload PORT=<node>`.
- `make engine` from repo root, or `cargo run` from `engine/`.
- `make lsp` = compile with `--only-compilation-database --build-path ./build`, then a `jq` rewrite that maps `build/sketch/` → `device/`, renames `.ino.cpp` → `.ino`, injects `-x c++` on `.ino` files, and strips `-MMD`/`-o`/object args. The `.clangd` compiler/sysroot override handles toolchain include discovery.

## Code Conventions & Common Patterns

**C++ (device/)**
- Classes `PascalCase` (`Tracker`, `Ekf6Dof`, `Mpu6050`, `StillDetector`, `UsbLink`, `Display`); methods/members `camelCase`; private members trailing `_` (`quat_`, `pktSeq_`, `lastTickUs_`).
- Constants: `UPPER_SNAKE` for macros/constexpr (`SDA_PIN`, `FUSION_RATE_HZ`, `EKF_GATE_MAHA`); `k`-prefixed class-private `static constexpr` (`kWinTicks`, `kAlpha`).
- **Error handling:** deterministic fail-stop — `setup()` hangs in `while(true) delay()` if MPU init fails; no soft degradation. NVS accel-cal is fault-tolerant (missing → identity defaults, silent no-op). CAL6 is abort-safe (only the SAVE writes NVS). NaN treated as failure via `!(x < th)` / `== 0` guards.
- **Filter tuning:** EKF sigmas/gates live in `config.h`; comments say to tune constants rather than touch the filter code.
- `config.h` display pins are **FIXED** (`TFT_MOSI=11 TFT_SCLK=12 TFT_CS=10 TFT_DC=9 TFT_RST=8`) with extensive comments on hardware traps — do not move them. `TFT_BL=7` is driven HIGH once in `Display::begin()` (backlight on).

- Minimal crate: no `lib.rs`, no modules beyond `tracking`, no async, no serde — just Bevy + `crossbeam-channel` + `serialport` + `jpeg-encoder`.
- Bevy idioms: `#[derive(Resource)]` / `#[derive(Component)]`, `Plugin` trait, function-based `app.add_systems(Startup/Update, ...)`, `Single<&mut Transform, With<LookRig>>`, `Local<T>` for per-frame scratch.
- Crash-fast errors: `unwrap_or_else(|e| panic!(...))` on serial open/clone; reader thread drops malformed frames, pose queue oldest-drops when full. No `Result` ergonomics / `thiserror` / `anyhow`.
- Explicit little-endian parsing (`f32::from_le_bytes`, `u32::from_le_bytes`), `#[inline]` micro-helpers.
- Comments are terse, explain *why* (e.g. drain-to-latest is not packet loss at ~4 packets/frame).

## Important Files

- `device/config.h` — every compile-time constant and the wire-contract sizes. The file any hardware/fusion change lands in.
- `device/packet.h` — canonical 36 B pose payload + `build_quat_packet()`; the byte contract's single source (host-testable).
- `device/frame.h` — framed-CDC contract (magic/type/len/CRC + `FrameParser`); parsed by `video_task.cpp` and the engine reader.
- `device/src/display/video.h` — the JPEG fragment layout + `VideoFragAssembler`; the video wire contract's single source (host-testable).
- `device/src/display/video_task.cpp` — the only `Serial` reader and the only panel writer: core-0 RX, JPEG reassembly/decode, blit.
- `device/device.ino` — the boot + loop sequencing (init → calib → CAL6 gate → start → stream).
- `device/src/position/tracker.cpp` — the per-tick pipeline (scheduler, axis remap, still-detect nudge, EKF predict/update, display-quat wrap).
- `engine/src/tracking.rs` — the packet parser, prediction/smoothing, MJPEG sender (encode → fragment → paced write), and camera rig.
- `Makefile` — the whole build/dev entry surface.

## Runtime / Tooling Preferences

- **Device toolchain:** `arduino-cli` with ESP32 core (had `3.3.11` at `~/.arduino15/packages/esp32/hardware/esp32/`), xtensa toolchain `esp-x32/2601`. Not PlatformIO.
- **Secrets:** `device/secrets.h` is **gitignored and unused by the firmware** (no includer; transport is USB-CDC, no WiFi). If it exists locally, never commit it.
- **`compile_commands.json` is generated** (`make lsp`); never hand-edit it. Regenerate after changing a sketch source or arduino-cli options — it is what makes clangd handle `.ino` files (via injected `-x c++`).
- `RFC 2119`-style MUSTs above follow from these facts.

## Contribution Workflow

Every change lands through a **branch → PR → review → merge** loop. Never commit
directly to `main`. This is the only merge path, and it applies to every
code/document change an agent makes here.

1. **New branch** from up-to-date `main` — `git checkout main && git pull && git checkout -b <topic>`.
   Use a short `topic` that names the change (the repo uses numbered `cycle-N-...`
   branches for sequenced work, e.g. `cycle6-tests`); keep the branch
   single-purpose.
2. **Commit** atomic, one logical change per commit. `device/secrets.h` is
   gitignored — never stage it. Verify before committing:
   - device change → `make compile`
   - engine change → `cargo build` (or `make engine`)
   - wire-contract / test change → `make test`
3. **Push** the branch to `origin` (remote: `git@github.com:Quentin2006/6-DoF-motion-tracking-puck.git`).
4. **Open a PR** against `main`. Title with a per-area prefix (`Device:`, `Engine:`,
   `Test:`); body states what changed, why, and the exact verification commands +
   results. Never include generated artifacts (`build/`, `target/`,
   `compile_commands.json`) — they're gitignored anyway.
5. **Review** the PR — self-review for correctness and against `# Code
   Conventions`, especially anything that touches the cross-file wire contract
   (`device/packet.h` ↔ `engine/src/tracking.rs`), which must stay in lockstep.
6. **Merge** back to `main` once the review is clean and the checks pass. Prefer a
   readable history; merge one PR at a time and keep `main` linear.

Multi-step efforts are split into dependency-ordered PRs (one logical change
each) and merged sequentially — never batch unrelated changes. The device↔engine
wire contract is the one thing that can break silently: if either half changes
the packet format, `make test` is the gate.

**Secrets stay local.** `device/secrets.h` is gitignored and unused (no WiFi
transport); never commit it if it exists locally.

## Testing & QA

**No hardware-dependent unit harness and no full-firmware host test.** The only
automated tests guard the cross-half packet contract; functional/IMU
verification stays manual.

- **`make test`** runs both halves against the same committed fixture:
  1. **Engine parser test** (`engine/src/tracking.rs`, `#[cfg(test)]`) — decode
     `tests/packet_fixture.txt` and assert `parse_pose` recovers the logical values
     (consumer half).
  2. **C++ producer test** (`tests/packet_test.cpp`, host `g++`) — build a v2
     packet with `build_quat_packet` and assert the bytes equal the same fixture
     (producer half). A framing test (`tests/frame_test.cpp`) covers garbage
     recovery, CRC rejection, split reassembly, and magic-in-payload.
  3. **Video fragment test** (`tests/video_test.cpp`, host `g++` + ASAN) — the
     assembler's ordering/loss/cap rules and the pinned wire bytes of one
     `FRAME_VIDEO` fragment, which the Rust test
     (`tracking.rs::video_fragment_wire_layout_is_pinned`) asserts as the same
     literal. ASAN is what makes "never writes past the assembly buffer" real.
  Both packet halves read the **same** `tests/packet_fixture.txt`, so a one-sided
  layout change fails its own test. Needs a host `g++` (not required to flash
  firmware).

- **CI (`.github/workflows/ci.yml`)** runs `cargo test` on the engine and
  `arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc device` on the
  firmware (same FQBN as the Makefile; it also installs the Adafruit display libs
  and JPEGDEC). No secrets step: the firmware compiles without `device/secrets.h`.

**Still manual / no automation:**
- **Compile gate** locally: `make compile` — keep green after any `device/` change.
- **On-device:** `make upload` + `make monitor` = the interactive loop. The serial
  CAL6 procedure (`Tracker::maybeRunAccelCal` / `runAccelCal` in `src/position/tracker.cpp`)
  calibrates the accelerometer (6 poses); `src/position/still_detect.cpp` re-biases
  gyro while still. The onboard LED goes red→green once streaming; the 1 Hz
- **Host render:** `make engine` opens the puck serial port (native USB
  `303a:1001` @921600, or `PUCK_SERIAL=<port>` to pin one) and renders the rig.

There is **no** unit coverage for `Ekf6Dof`, `StillDetector`, or `Mpu6050` — they
are hardware-coupled and the serial/calibration path is their real test. Add a
test only where it defends an observable contract and a plausible bug would fail
it (packet layout, axis mapping, calibration abort-safety), not plumbing.
