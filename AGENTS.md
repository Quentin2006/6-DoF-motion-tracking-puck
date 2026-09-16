# Repository Guidelines

## Project Overview

A 6-DoF motion-tracking puck. An ESP32-S3 puck reads an MPU6050 IMU, fuses
accelerometer + gyroscope into a 6-DoF orientation quaternion with a 6-state
EKF, and streams the pose over UDP (~250 Hz) to a Rust/Bevy desktop engine.
The engine binds a UDP socket, parses the packets, and drives a **look-around
camera rig** — the puck's orientation maps directly to camera rotation, so the
user moves the puck and the 3D view follows.

Two independent halves, no shared code, wired together by a single root
`Makefile` and hand-maintained constant agreement:

- `device/` — ESP32-S3 Arduino firmware (C++ / `.ino` + `.cpp`, `arduino-cli`).
- `engine/` — Rust **Bevy 0.19.1** desktop visualizer (`cargo run`).

## Architecture & Data Flow

```
 MPU6050 (I2C SDA=5 SCL=4 @400kHz)
   │  readGyro() dps + readAccel() m/s²   (mpu6050.cpp: cal offset/scale + bias)
   ▼
 Tracker::step(dt)  (tracker.cpp)
   │  selAxis(FUSION_AXIS_MAP) → body frame; body-accel norm
   ├─ StillDetector::update(driver-frame gyro + norm) → Mpu6050.nudgeGyroBias (clamped 0.002 dps/tick)
   ├─ Ekf6Dof::predict(gyro rad/s) → Ekf6Dof::updateAccel(accel)  [gated → gated_++]
   └─ wrap to display quat  q_disp = QW_ALIGN * q_ekf * QB_ALIGN   (right-multiply)
   ▼
 device.ino loop() → Net::sendQuat(quat, gyro, micros())
   │  36-byte packet: seq u32 | stamp u32 | quat xyzw f32 | gyro xyz f32  (all LE)
   ▼
 engine/src/tracking.rs::update  (UDP 0.0.0.0:3333, drain-to-latest)
   │  parse v1 (24 B) / v2 (36 B) → predict forward (PREDICT_TAU_S) → slerp smooth (SMOOTH_TAU_S)
   ▼
 LookRig camera Transform.rotation
```

**Key modules (`device/`)**

| File | Role |
|---|---|
| `device.ino` | Orchestrator: `setup()` init/calib, `loop()` fixed `dt = 1/500` step + 1 s status line |
| `config.h` | Single source of all compile-time constants (pins, FSR, gravity, UDP port, all EKF sigmas/gates, axis map, display pins) |
| `tracker.{h,cpp}` | Pipeline glue + `micros()` scheduler; owns `Mpu6050`, `Ekf6Dof`, `StillDetector` |
| `mpu6050.{h,cpp}` | I2C driver + NVS accel-cal persistence (`Preferences`, namespace `mpu`, keys `ax0-2`/`as0-2`) |
| `fusion.{h,cpp}` | `Ekf6Dof` — 6-state EKF (quat w,x,y,z + gyro bias) |
| `still_detect.{h,cpp}` | `StillDetector` — gyro re-bias while stationary (50-tick window, variance/norm gates) |
| `net.{h,cpp}` | WiFi STA + `WiFiUDP`, 36 B packet, alternator for ~250 Hz |
| `packet.h` | Canonical 36 B UDP wire layout + `build_quat_packet()` — host-compilable, single source of the byte contract |
| `cal.{h,cpp}` | `AccelCalibrator` — one-time serial-guided 6-pose (`CAL6`) accel calibration |
| `led.{h,cpp}` | `StatusLed` — onboard WS2812: red = calibrating/booting, green = streaming |
| `secrets.h` / `secrets.h.example` | WiFi credentials (`WIFI_SSID`, `WIFI_PASSWORD`, `ENGINE_IP`) |

**Key module (`engine/`)**

| File | Role |
|---|---|
| `src/main.rs` | `App::new().add_plugins(DefaultPlugins).add_plugins(TrackPlugin).run()` |
| `src/tracking.rs` | All behavior: `Socket` resource, `LookRig` marker, `TrackPlugin`, `update` ECS system |

**Cross-file wire contract (edit both halves together).** `device/config.h` and
`engine/src/tracking.rs` hard-code the same UDP protocol by hand; there is no
shared crate or serializer. `UDP_PORT = 3333` and packet sizes `24` (v1, no
gyro) / `36` (v2, +gyro) **must match in both files**. `config.h` even notes
"must match engine/src/tracking.rs UDP_PORT". Both x86 and ESP32 are
little-endian, so no byte-swapping is done.

The packet byte layout is **canonical in `device/packet.h`**
(`build_quat_packet`, host-compilable, no Arduino deps); `net.cpp` calls it and
`config.h` aliases its length constants. The committed `tests/packet_fixture.txt`
is the single source of truth and is asserted by **both** halves — the C++ host
test (`tests/packet_test.cpp`, producer) and the Rust parser tests
(`tracking.rs`, consumer). Changing the packet layout means updating
`device/packet.h`, `tests/packet_fixture.txt`, and the parser — any one-sided
edit fails `make test`.

## Key Directories

- `device/` — ESP32-S3 Arduino sketch (source of truth for firmware).
- `engine/` — Rust Bevy visualizer (`src/main.rs`, `src/tracking.rs`).
- `tests/` — cross-contract tests: `packet_fixture.txt` (shared golden bytes), `packet_test.cpp` (C++ producer vs fixture).
- `.github/` — CI (`workflows/ci.yml`): engine `cargo test` + device `arduino-cli compile`.
- `build/` — arduino-cli output incl. raw `build/compile_commands.json` (gitignored).
- Root datasheets `ST7789VW.PDF` (display), `MPU-6000.PDF` (IMU) — reference only.
- No `docs/`, `scripts/`; all test infra lives in `tests/` + CI.

## Development Commands

Everything is driven from the root `Makefile`. Two real, authoritative build
chains: **`arduino-cli`** (device) and **`cargo`** (engine). `clangd`/`.clangd`
is editor IntelliSense only — it does not compile or flash.

```bash
make compile          # build firmware only: arduino-cli compile --fqbn esp32:esp32:esp32s3 device
make upload           # flash to /dev/ttyACM0: arduino-cli upload --fqbn $(FQBN) -p /dev/ttyACM0 device
make monitor          # serial monitor @115200 (uses device/sketch.yaml port/baud)
make device           # compile + upload + monitor  (interactive loop)
make engine           # cd engine && cargo run      (host Bevy visualizer)
make all              # compile + upload + engine   (full pipeline)
make clean            # arduino-cli compile --clean device
make lsp              # regenerate + rewrite compile_commands.json for clangd
make test             # cargo test (engine) + host packet-contract test (device/packet.h vs tests/packet_fixture.txt)
```

- **FQBN** default `esp32:esp32:esp32s3` (in `Makefile` `?=` and `device/sketch.yaml`); override with `make compile FQBN=<other>`.
- **Port** `/dev/ttyACM0`, **baud** 115200.
- `make engine` from repo root, or `cargo run` from `engine/`.
- `make lsp` = compile with `--only-compilation-database --build-path ./build`, then a `jq` rewrite that maps `build/sketch/` → `device/`, renames `.ino.cpp` → `.ino`, injects `-x c++` on `.ino` files, and strips `-MMD`/`-o`/object args. The `.clangd` compiler/sysroot override handles toolchain include discovery.

## Code Conventions & Common Patterns

**C++ (device/)**
- `#pragma once` (never include guards); lowercase `.h`/`.cpp` (except `device.ino`).
- Classes `PascalCase` (`Tracker`, `Ekf6Dof`, `Mpu6050`, `StillDetector`, `Net`, `AccelCalibrator`, `StatusLed`); methods/members `camelCase`; private members trailing `_` (`quat_`, `pktSeq_`, `lastTickUs_`).
- Constants: `UPPER_SNAKE` for macros/constexpr (`SDA_PIN`, `FUSION_RATE_HZ`, `EKF_GATE_MAHA`); `k`-prefixed class-private `static constexpr` (`kWinTicks`, `kAlpha`).
- `enum class Step { Skip, Processed, Failed }` (scoped).
- 2-space indent; braces same line; space inside parens `if (cond)`. Avoid C-style casts.
- **Error handling:** deterministic fail-stop — `setup()` hangs in `while(true) delay()` if MPU init or WiFi fails; no soft degradation. NVS accel-cal is fault-tolerant (missing → identity defaults, silent no-op). CAL6 is abort-safe (only the SAVE writes NVS). NaN treated as failure via `!(x < th)` / `== 0` guards.
- **Filter tuning:** EKF sigmas/gates live in `config.h`; comments say to tune constants rather than touch the filter code.
- `config.h` display pins are **FIXED** (`CLK=18 CS=8 DIN=3 DC=6 RST=9`, `BL=-1` rail-tied) with extensive comments on hardware traps — do not move them, and `BL` must never be driven as GPIO.

**Rust (engine/)**
- Minimal crate: no `lib.rs`, no modules beyond `tracking`, no async, no serde — just Bevy + `std::net::UdpSocket`.
- Bevy idioms: `#[derive(Resource)]` / `#[derive(Component)]`, `Plugin` trait, function-based `app.add_systems(Startup/Update, ...)`, `Single<&mut Transform, With<LookRig>>`, `Local<T>` for per-frame scratch.
- Crash-fast errors: `unwrap_or_else(|e| panic!(...))` on bind, `expect()` on nonblocking, `match { _ => {} }` to drop malformed packets. No `Result` ergonomics / `thiserror` / `anyhow`.
- Explicit little-endian parsing (`f32::from_le_bytes`, `u32::from_le_bytes`), `#[inline]` micro-helpers.
- Comments are terse, explain *why* (e.g. drain-to-latest is not packet loss at ~4 packets/frame).

## Important Files

- `device/config.h` — every compile-time constant and the wire-contract sizes. The file any hardware/fusion change lands in.
- `device/packet.h` — canonical UDP packet layout + `build_quat_packet()`; the byte contract's single source (host-testable).
- `device/device.ino` — the boot + loop sequencing (init → calib → CAL6 gate → seedEKF → stream).
- `device/tracker.cpp` — the per-tick pipeline (scheduler, axis remap, still-detect nudge, EKF predict/update, display-quat wrap).
- `engine/src/tracking.rs` — the packet parser, prediction/smoothing, and camera rig.
- `Makefile` — the whole build/dev entry surface.
- `device/secrets.h.example` — template to create your local `device/secrets.h`.

## Runtime / Tooling Preferences

- **Device toolchain:** `arduino-cli` with ESP32 core (had `3.3.11` at `~/.arduino15/packages/esp32/hardware/esp32/`), xtensa toolchain `esp-x32/2601`. Not PlatformIO.
- **Engine toolchain:** Rust `edition 2024`, `cargo`; `[profile.dev] opt-level = 1`, `[profile.dev.package."*"] opt-level = 3`.
- **Secrets:** `device/secrets.h` is **gitignored and must never be committed**; `config.h` includes it unconditionally, so a build without a populated `secrets.h` fails deliberately. Create it from `secrets.h.example`. It currently holds real credentials in the working tree (SSID/password/`ENGINE_IP`) — never print or commit them.
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

**Secrets stay local.** `device/secrets.h` is never committed; CI and local
builds generate it from `secrets.h.example` when a compile needs it.

## Testing & QA

**No hardware-dependent unit harness and no full-firmware host test.** The only
automated tests guard the cross-half packet contract; functional/IMU
verification stays manual.

- **`make test`** runs both halves against the same committed fixture:
  1. **Engine parser tests** (`engine/src/tracking.rs`, `#[cfg(test)]`) — decode
     `tests/packet_fixture.txt` and assert `parse_v1` / `parse_v2` / `header`
     recover the logical values (consumer half).
  2. **C++ producer test** (`tests/packet_test.cpp`, host `g++`) — build a v2
     packet with `build_quat_packet` and assert the bytes equal the same fixture
     (producer half).
  Both read the **same** `tests/packet_fixture.txt`, so a one-sided layout change
  fails its own test. Needs a host `g++` (not required to flash firmware).

- **CI (`.github/workflows/ci.yml`)** runs `cargo test` on the engine and
  `arduino-cli compile --fqbn esp32:esp32:esp32s3 device` on the firmware. The
  device job copies `device/secrets.h.example` → `device/secrets.h` first
  (config.h includes it unconditionally); real secrets are never committed.

**Still manual / no automation:**
- **Compile gate** locally: `make compile` — keep green after any `device/` change.
- **On-device:** `make upload` + `make monitor` = the interactive loop. The serial
  CAL6 procedure (`device/cal.cpp`) calibrates the accelerometer (6 poses);
  `device/still_detect.cpp` re-biases gyro while still. Status LED goes red→green
  once streaming; `q`/`seq`/`tick`/`over`/`gate`/`bg`/`st` line prints each second.
- **Host render:** `make engine` binds UDP `0.0.0.0:3333` and renders the rig.

There is **no** unit coverage for `Ekf6Dof`, `StillDetector`, or `Mpu6050` — they
are hardware-coupled and the serial/calibration path is their real test. Add a
test only where it defends an observable contract and a plausible bug would fail
it (packet layout, axis mapping, calibration abort-safety), not plumbing.
