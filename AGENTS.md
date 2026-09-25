# Repository Guidelines

## Project Overview

A 6-DoF motion-tracking puck. An ESP32-S3 puck reads an MPU6050 IMU, fuses
accelerometer + gyroscope into a 6-DoF orientation quaternion with a 6-state
EKF, and streams framed pose packets (~250 Hz) over a native USB TinyUSB
vendor-bulk link to a Rust/Bevy desktop engine. The engine claims that one USB
interface, parses the frames, drives a **look-around camera rig**, and sends
the rendered stereo pair back over the same bulk interface. The CH343 COM/UART
bridge is separate and remains dedicated to flashing, logs, and calibration.

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
device.ino loop() → BulkLink::sendQuat(quat, gyro)
   │  FRAME_POSE frame: magic 0x55 0xAA | type u8 | len u16 LE | 36 B payload | crc u16 LE
   │  payload: seq u32 | stamp u32 | quat xyzw f32 | gyro xyz f32  (all LE)
   ▼
 engine/src/tracking.rs::update  (USB vendor bulk 303a:1002, drain-to-latest)
   ▼
 LookRig camera Transform.rotation
```

Video runs the other way down the same bulk interface, at its own pace:

```
 Bevy 640x240 RGBA render target = stereo pair (left eye | right eye viewports)
   │  readback → host-side dirty rectangle per eye (16-px aligned bounding box)
   │  jpeg-encoder q75 4:2:0, one JPEG per rectangle, concatenated in a blob
   │  FRAME_VIDEO_RECT frame: 4 B header + 11 B per rect (eye/x/y/w/h/jpeg_len)
   │  FRAME_VIDEO fragments: 8 B header (frame/frag/count/len) + <= 1280 B of blob
   │  one fragment per slice of the 25 ms frame (never a burst: bulk OUT FIFO is 64 B)
   ▼
 device/src/display/video_task.cpp  (core-0 task)
   │  VideoFragAssembler → JPEGDEC per rectangle → Display::blitTile per 16x16 tile
   ▼
 two ST7789 panels, one per eye (CS=10 left, CS=13 right) @ 40 fps
```

Sending only the changed rectangles is what makes the frame rate: the device
decodes ~1 ms of JPEG per typical frame instead of the ~17 ms a 640x240 canvas
costs, and the SPI blit follows the dirty area (2x153600 B at the measured
3.38 MB/s = 90.8 ms if it ever had to paint both panels in full, which is why a
keyframe is the only thing that does). Measured: 45 dirty tiles of ~45 tiles of
rectangle area per frame, i.e. the bounding boxes are tight, and the blit — at
0.17 ms per 16x16 tile — is what is left of the budget.

40 fps is the measured painted capacity, not a transport promise: each frame
spends roughly 12-15 ms decoding/blitting dirty rectangles plus 3-4 ms reading
and parsing. The vendor-bulk cutover leaves the 1,280-byte fragments and 25-ms
pacing unchanged, so extra USB bandwidth does not raise this ceiling. Sending
the same per-frame work at 60 fps still exceeds the device budget; benchmark
throughput is raw transport headroom, not a 60-fps panel guarantee.

**Key modules (`device/`)**

| File | Role |
|---|---|
| `device.ino` | Orchestrator: `setup()` `videoTaskStart()` (core-0 bulk init) → init/calib → `videoTaskGo()`, `loop()` (tick → send → UART0 report → video counters) |
| `config.h` | Single source of compile-time constants (pins, FSR, gravity, USB VID/PID, EKF sigmas/gates, axis map, display pins) |
| `src/position/tracker.{h,cpp}` | Pipeline glue + `micros()` scheduler + CAL6 accelerometer calibration + 1 Hz status line; owns `Mpu6050`, `Ekf6Dof`, `StillDetector` |
| `src/position/mpu6050.{h,cpp}` | I2C driver + NVS accel-cal persistence (`Preferences`, namespace `mpu`, keys `ax0-2`/`as0-2`) |
| `src/position/fusion.{h,cpp}` | `Ekf6Dof` — 6-state EKF (quat w,x,y,z + gyro bias) |
| `src/position/still_detect.{h,cpp}` | `StillDetector` — gyro re-bias while stationary (50-tick window, variance/norm gates) |
| `src/transport/usb_bulk.{h,cpp}` | TinyUSB vendor descriptor, 64-byte chunk queue, bulk read/write/flush, and interval transport statistics |
| `link.{h,cpp}` | `BulkLink` pose producer: complete `FRAME_POSE` writes + sequence counter |
| `frame.h` | Framed bulk byte-stream contract (magic/type/len/CRC) — host-compilable, parsed by `link.cpp`/`video_task.cpp` and `engine/src/tracking.rs` |
|`src/display/display.{h,cpp}`|Two-panel ST7789 driver: `blitTile(panel, x, y, px)` picks the chip select per tile, one shared SPI transaction per frame, idle RGB cycle on both panels|
|`src/display/video_task.{h,cpp}`|Core-0 task owning `BulkUsb` receive: rectangle table + blob reassembly, JPEGDEC per rectangle + tile blit, 1 Hz display/transport counters|
|`src/display/video.h`|Video wire contract: stereo canvas geometry, the dirty-rectangle table + `parse_video_rects_hdr`, the JPEG fragment header + `VideoFragAssembler` — host-compilable, mirrored by `engine/src/tracking.rs`|
| `packet.h` | Canonical 36 B pose payload + `build_quat_packet()` — host-compilable, single source of the byte contract |

**Key modules (`engine/`)**

| File | Role |
|---|---|
| `src/main.rs` | Dispatches the optional raw benchmark, then starts the Bevy app |
|`src/usb_bulk.rs`|One claimed `rusb` handle behind `Arc`, descriptor endpoint selection, timed bulk I/O, and the 10-second benchmark|
|`src/tracking.rs`|`PuckLink`/`VideoTx` resources, `LookRig` stereo cameras, `TrackPlugin`, pose update/recenter, bulk reader, dirty-rectangle JPEG sender, and video logs|

**Cross-file wire contract (edit both halves together).** `device/frame.h`,
`device/packet.h`, `device/src/display/video.h` and `engine/src/tracking.rs`
hard-code the same framed bulk protocol by hand; there is no shared crate or
serializer. The 36-byte pose payload (`QUAT_PACKET_LEN_V2`) inside a
`FRAME_POSE` frame, the `FRAME_VIDEO_RECT` rectangle table, and the
640×240 baseline-JPEG stereo-pair fragments (`FRAME_VIDEO`), **must match in
both halves**. Both x86 and ESP32 are little-endian, so no byte-swapping is
done.

The fragment layout is **canonical in `device/src/display/video.h`**
(`VideoFragAssembler`, host-compilable, no Arduino deps; the engine's
`send_blob` is the producer). `tests/video_test.cpp` and
`tracking.rs::video_fragment_wire_layout_is_pinned` assert the same literal
bytes for one fragment, so a one-sided layout change fails `make test`.

The rectangle table is canonical there too (`VIDEO_W = 640` split into two
320-wide eye halves, 16x16 tile grid, up to `VIDEO_RECTS_MAX = 4` rectangles,
`parse_video_rects_hdr`, and the eye-local rect that names its own panel). A
frame is a rect table followed by the blob its `jpeg_len`s slice up; the device
paints a table only when the lengths match the assembled blob and the rectangle
decodes at exactly its declared size, so a mismatched frame is counted bad
rather than painted somewhere wrong.

The host re-sends the whole canvas every `VIDEO_KEYFRAME_MS` (2 s) — with
rectangles the device holds no full-canvas pixels, so that periodic repair is
what fixes the panels after a lost frame or after the device's own idle RGB
cycle. It is rolled over `VIDEO_KEYFRAME_STEPS` (4) frames, one full-height
slice per eye each, plus that frame's dirty rectangles: the whole canvas is 600
tiles of SPI (~102 ms measured, five times a 40 fps frame budget) and sending it
in one frame made the panel hitch for 144 ms every two seconds. Rolled, the
worst repair frame measured 50-57 ms instead. The engine's
`send_rects`/`dirty_rects` are the producer, and
`tracking.rs::video_rect_frame_wire_layout_is_pinned` + `dirty_rects_cover_exactly_the_changed_tiles`
assert the same bytes and the same rectangle bounds as `tests/video_test.cpp`.

The packet byte layout is **canonical in `device/packet.h`**
(`build_quat_packet`, host-compilable, no Arduino deps); `link.cpp`
(`BulkLink::sendQuat`) calls it and names `QUAT_PACKET_LEN_V2` directly. The committed `tests/packet_fixture.txt`
is the single source of truth and is asserted by **both** halves — the C++ host
test (`tests/packet_test.cpp`, producer) and the Rust parser test
(`tracking.rs`, consumer). Changing the packet layout means updating
`device/packet.h`, `tests/packet_fixture.txt`, and the parser — any one-sided
edit fails `make test`.

## Key Directories

- `device/` — ESP32-S3 Arduino sketch (source of truth for firmware).
- `engine/` — Rust Bevy visualizer (`src/main.rs`, `src/tracking.rs`).
- `tests/` — cross-contract tests: `packet_fixture.txt` (shared golden bytes), `packet_test.cpp` (C++ producer vs fixture), `frame_test.cpp` (framing/CRC/desync), `video_test.cpp` (JPEG fragment + dirty-rectangle contract, ASAN).
- `.github/` — CI (`workflows/ci.yml`): engine `cargo test` + device `arduino-cli compile`.
- `build/` — arduino-cli output incl. raw `build/compile_commands.json` (gitignored).
- Root datasheets `ST7789VW.PDF` (display), `MPU-6000.PDF` (IMU) — reference only.
- No `docs/`, `scripts/`; all test infra lives in `tests/` + CI.

## Development Commands

Everything is driven from the root `Makefile`. Two real, authoritative build
chains: **`arduino-cli`** (device) and **`cargo`** (engine). `clangd`/`.clangd`
is editor IntelliSense only — it does not compile or flash.

```bash
make usb-permissions   # one-time udev install/reload for native 303a:1002
make compile           # arduino-cli compile with explicit USB-OTG TinyUSB FQBN
make upload            # flash to CH343 $(PORT); PORT=<node> overrides
make monitor           # CH343 UART0 logs and CAL6 interaction
make device            # compile + CH343 upload + CH343 monitor
make engine            # native USB vendor-bulk Bevy visualizer
make all               # compile + CH343 upload + bulk engine
make clean             # arduino-cli compile --clean --fqbn $(FQBN) device
make lsp               # regenerate + rewrite compile_commands.json for clangd
make test              # Rust + packet/framing/video host contract tests
```

- **FQBN** default `esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=default,UploadMode=default` (in `Makefile` and `device/sketch.yaml`); override with `make compile FQBN=<other>`.
- **Native link:** the engine claims exact USB `303a:1002`, auto-detaches any kernel driver, and selects the unique class-`0xff` interface with one bulk IN/OUT pair. `make usb-permissions` installs `engine/udev/99-puck-usb.rules`; replug native USB afterward.
- **Console/flashing:** `PORT` prefers `/dev/serial/by-id/usb-1a86_USB_Single_Serial_*-if00`, falls back to `/dev/ttyACM0`, and is independent of the native bulk device.
- `make engine` from repo root, or `cargo run` from `engine/`.
- `make lsp` = compile with `--only-compilation-database --build-path ./build`, then a `jq` rewrite that maps `build/sketch/` → `device/`, renames `.ino.cpp` → `.ino`, injects `-x c++` on `.ino` files, and strips `-MMD`/`-o`/object args. The `.clangd` compiler/sysroot override handles toolchain include discovery.

## Code Conventions & Common Patterns

**C++ (device/)**
- Classes `PascalCase` (`Tracker`, `Ekf6Dof`, `Mpu6050`, `StillDetector`, `BulkLink`, `BulkUsb`, `Display`); methods/members `camelCase`; private members trailing `_` (`quat_`, `pktSeq_`, `lastTickUs_`).
- Constants: `UPPER_SNAKE` for macros/constexpr (`SDA_PIN`, `FUSION_RATE_HZ`, `EKF_GATE_MAHA`); `k`-prefixed class-private `static constexpr` (`kWinTicks`, `kAlpha`).
- **Error handling:** deterministic fail-stop — `setup()` hangs in `while(true) delay()` if MPU init fails; no soft degradation. NVS accel-cal is fault-tolerant (missing → identity defaults, silent no-op). CAL6 is abort-safe (only the SAVE writes NVS). NaN treated as failure via `!(x < th)` / `== 0` guards.
- **Filter tuning:** EKF sigmas/gates live in `config.h`; comments say to tune constants rather than touch the filter code.
- `config.h` display pins are **FIXED** (`TFT_MOSI=11 TFT_SCLK=12 TFT_CS=10 TFT_DC=9 TFT_RST=8`) with extensive comments on hardware traps — do not move them. `TFT_BL=7` is driven HIGH once in `Display::begin()` (backlight on).

- Minimal crate: no `lib.rs`, no async, no serde — just Bevy + `crossbeam-channel` + `rusb` + `jpeg-encoder`.
- Bevy idioms: `#[derive(Resource)]` / `#[derive(Component)]`, `Plugin` trait, function-based `app.add_systems(Startup/Update, ...)`, `Single<&mut Transform, With<LookRig>>`, `Local<T>` for per-frame scratch.
- Crash-fast startup: `BulkLink::open()` panics with the exact USB/interface/endpoint error; reader timeouts idle normally, other I/O failures stop the affected worker. The pose queue oldest-drops when full. No `thiserror` / `anyhow`.
- Explicit little-endian parsing (`f32::from_le_bytes`, `u32::from_le_bytes`), `#[inline]` micro-helpers.
- Comments are terse, explain *why* (e.g. drain-to-latest is not packet loss at ~4 packets/frame).

## Important Files

- `device/config.h` — every compile-time constant and the wire-contract sizes. The file any hardware/fusion change lands in.
- `device/packet.h` — canonical 36 B pose payload + `build_quat_packet()`; the byte contract's single source (host-testable).
- `device/frame.h` — framed bulk contract (magic/type/len/CRC + `FrameParser`); parsed by `video_task.cpp` and the engine reader.
- `device/src/display/video.h` — the stereo canvas geometry, the dirty-rectangle table + parser, and the JPEG fragment layout + `VideoFragAssembler`; the video wire contract's single source (host-testable).
- `device/src/transport/usb_bulk.cpp` — TinyUSB interface registration, static chunk queue, host-to-device RX accounting, complete-frame TX, and interval stats.
- `device/src/display/video_task.cpp` — the only `BulkUsb` reader and the only panel writer: core-0 RX, rectangle table + blob reassembly, per-rectangle JPEG decode, per-tile blit.
- `device/device.ino` — the boot + loop sequencing (bulk init → UART0 calibration → display ready → stream).
- `engine/src/usb_bulk.rs` — shared `rusb` handle, descriptor endpoint selection, timed bulk I/O, and raw benchmark.
- `engine/src/tracking.rs` — the packet parser, prediction/smoothing, MJPEG sender (encode → fragment → paced write), and camera rig.
- `Makefile` — the whole build/dev entry surface.

## Runtime / Tooling Preferences

- **Device toolchain:** `arduino-cli` with ESP32 core (had `3.3.11` at `~/.arduino15/packages/esp32/hardware/esp32/`), xtensa toolchain `esp-x32/2601`. Not PlatformIO.
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
2. **Commit** atomic, one logical change per commit. Verify before committing:
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
  3. **Video test** (`tests/video_test.cpp`, host `g++` + ASAN) — the
     assembler's ordering/loss/cap rules, the pinned wire bytes of one
     `FRAME_VIDEO` fragment and of one `FRAME_VIDEO_RECT` table, and every bound
     the rect parser must refuse (unknown eye, off-grid x/y/w/h, a rect past the
     panel edge, a zero-length JPEG, a truncated table, a blob over the
     assembly cap). The Rust tests (`video_fragment_wire_layout_is_pinned`,
     `video_rect_frame_wire_layout_is_pinned`,
     `dirty_rects_cover_exactly_the_changed_tiles`,
     `crop_rgba_packs_the_rectangle_rows_tight`) assert the same literals, the
     same rectangle bounds, and that a crop adds the eye's half of the canvas.
     ASAN is what makes "never writes past the assembly buffer" — and "never
     reads past a short payload" — real.
  Both packet halves read the **same** `tests/packet_fixture.txt`, so a one-sided
  layout change fails its own test. Needs a host `g++` (not required to flash
  firmware).

- **CI (`.github/workflows/ci.yml`)** runs `cargo test` on the engine and
  `arduino-cli compile --fqbn esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=default,UploadMode=default device` on the
  firmware (same FQBN as the Makefile; it installs libusb plus the Adafruit display libs
  and JPEGDEC).

**Still manual / no automation:**
- **Compile gate** locally: `make compile` — keep green after any `device/` change.
- **On-device:** `make upload` + `make monitor` use the CH343 COM/UART console. The serial CAL6 procedure (`Tracker::maybeRunAccelCal` / `runAccelCal` in `src/position/tracker.cpp`) calibrates the accelerometer (6 poses); `src/position/still_detect.cpp` re-biases gyro while still. The onboard LED goes red→green once streaming.
- **Host render:** `make engine` opens native USB `303a:1002` and renders the rig. Engine side: `video 40 fps ... txerr 0` and no `seq gap:` line. Device side over CH343: `vid 40 drop 0 bad 0 ... rxdrop 0`. `drop > 0` is an abandoned video frame, `bad > 0` is a frame the device refused, and `rxdrop > 0` is bulk RX queue overflow. `R` is the only keyboard binding: it recenters the view, so the puck's orientation at the press becomes the reference.

There is **no** unit coverage for `Ekf6Dof`, `StillDetector`, or `Mpu6050` — they
are hardware-coupled and the serial/calibration path is their real test. Add a
test only where it defends an observable contract and a plausible bug would fail
it (packet layout, axis mapping, calibration abort-safety), not plumbing.
