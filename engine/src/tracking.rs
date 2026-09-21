use std::io::{Read as _, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use bevy::camera::RenderTarget;
use bevy::prelude::*;
use bevy::render::gpu_readback::{Readback, ReadbackComplete};
use bevy::render::render_resource::{TextureFormat, TextureUsages};

// Marker for the camera rig driven by the tracking puck (look-around).
#[derive(Component)]
struct LookRig;

// must match device/packet.h QUAT_PACKET_LEN_V2 (pose payload inside a FRAME_POSE frame)
const PACKET_LEN: usize = 36;

// Framed CDC contract, mirroring device/frame.h (not a shared crate).
const FRAME_MAGIC_0: u8 = 0x55;
const FRAME_MAGIC_1: u8 = 0xAA;
const FRAME_POSE: u8 = 0x01;
const FRAME_VIDEO: u8 = 0x02; // one JPEG fragment: 8 B header + <= 1280 B
const FRAME_MAX_PAYLOAD: usize = 1288; // 8 + VIDEO_FRAG_MAX

// Transport+render delay extrapolated from packet gyro, and display smoothing.
const PREDICT_TAU_S: f32 = 0.008;
const SMOOTH_TAU_S: f32 = 0.006;

// Puck serial link. The S3 board exposes two USB ports: the USB-Serial-JTAG
// peripheral (Espressif 303a:1001 — the firmware's CDC `Serial`, where the
// frames are) and the on-board UART bridge (1a86:55d3 — flash/console only).
// Attach by VID/PID: the ttyACM index depends on plug order, the VID/PID does
// not. 921600 is the one baud that also works on the bridge; on USB-JTAG it is
// a no-op — the pipe is capped by the USB delivery path, not the line rate.
const PUCK_VID: u16 = 0x303a;
const PUCK_PID: u16 = 0x1001;
const SERIAL_BAUD: u32 = 921600;

// Video stream to the puck panel: 320x240 baseline JPEG, fragmented into
// FRAME_VIDEO frames. Must match device/src/display/video.h VIDEO_* constants.
const VIDEO_W: usize = 320;
const VIDEO_H: usize = 240;
const VIDEO_FRAG_MAX: usize = 1280; // JPEG bytes per fragment; 8 + this = FRAME_MAX_PAYLOAD
const VIDEO_JPEG_MAX: usize = 49152; // device assembly buffer, and the largest frame accepted
const JPEG_QUALITY: u8 = 75; // the one knob trading fidelity for frame rate
const VIDEO_TARGET_FPS: u64 = 30;
const VIDEO_HEARTBEAT_MS: u64 = 2000; // static scene: re-send so the device stays in video mode

struct Frame {
    rgba: Box<[u8]>, // 320x240 RGBA8 readback, pooled
    id: u16,
}

fn frame_crc_feed(mut crc: u16, b: u8) -> u16 {
    crc ^= (b as u16) << 8;
    for _ in 0..8 {
        crc = if crc & 0x8000 != 0 {
            (crc << 1) ^ 0x1021
        } else {
            crc << 1
        };
    }
    crc
}

// Build one frame (magic + type + len LE + payload + crc LE) into `out`.
// Caller must size `out` >= 7 + payload.len().
fn build_frame(out: &mut [u8], ty: u8, payload: &[u8]) -> usize {
    let len = payload.len() as u16;
    out[0] = FRAME_MAGIC_0;
    out[1] = FRAME_MAGIC_1;
    out[2] = ty;
    out[3] = (len & 0xFF) as u8;
    out[4] = ((len >> 8) & 0xFF) as u8;
    out[5..5 + payload.len()].copy_from_slice(payload);
    let mut crc: u16 = 0xFFFF;
    crc = frame_crc_feed(crc, ty);
    crc = frame_crc_feed(crc, out[3]);
    crc = frame_crc_feed(crc, out[4]);
    for &b in payload {
        crc = frame_crc_feed(crc, b);
    }
    out[5 + payload.len()] = (crc & 0xFF) as u8;
    out[5 + payload.len() + 1] = ((crc >> 8) & 0xFF) as u8;
    5 + payload.len() + 2
}

// Fragment one JPEG into FRAME_VIDEO frames and write them, spreading the
// fragments evenly over `interval` instead of bursting the whole image.
//
// The spread is not cosmetic: the puck's USB-Serial-JTAG RX FIFO is 64 B, so a
// burst makes its ISR miss packets and the frame arrives shredded (measured:
// 10.4 kB every 34 ms lost ~30% of its bytes; the same rate paced per fragment
// lost none). Fragment i leaves at frame_start + (i+1)*interval/count, so a
// frame still takes one interval even when encoding ate part of the first
// slice, and a late fragment just catches up.
fn send_jpeg<W: Write>(
    writer: &mut W,
    id: u16,
    jpeg: &[u8],
    frame_start: Instant,
    interval: Duration,
) {
    let count = jpeg.len().div_ceil(VIDEO_FRAG_MAX).max(1);
    let per_frag = interval / count as u32;
    let mut payload = [0u8; 8 + VIDEO_FRAG_MAX];
    let mut frame = [0u8; 7 + FRAME_MAX_PAYLOAD];
    for (i, part) in jpeg.chunks(VIDEO_FRAG_MAX).enumerate() {
        payload[0..2].copy_from_slice(&id.to_le_bytes());
        payload[2..4].copy_from_slice(&(i as u16).to_le_bytes());
        payload[4..6].copy_from_slice(&(count as u16).to_le_bytes());
        payload[6..8].copy_from_slice(&(part.len() as u16).to_le_bytes());
        payload[8..8 + part.len()].copy_from_slice(part);
        let n = build_frame(&mut frame, FRAME_VIDEO, &payload[..8 + part.len()]);
        let _ = writer.write_all(&frame[..n]);
        let due = frame_start + per_frag * (i as u32 + 1);
        if let Some(left) = due.checked_duration_since(Instant::now()) {
            thread::sleep(left);
        }
    }
}

#[derive(Default)]
struct FrameParser {
    state: u8, // 0 hunt, 1 got-magic0, 2 type, 3 len-lo, 4 len-hi, 5 payload, 6 crc-lo, 7 crc-hi
    ty: u8,
    len: usize,
    pos: usize,
    crc: u16,
    crc_rx: u16,
    buf: Vec<u8>,
}

impl FrameParser {
    // Returns Some((type, len)) exactly once per complete valid frame; the
    // payload is in `payload_out[..len]`. Bytes that do not form a frame
    // (log text) are consumed and ignored.
    fn push(&mut self, byte: u8, payload_out: &mut [u8]) -> Option<(u8, usize)> {
        match self.state {
            0 => {
                if byte == FRAME_MAGIC_0 {
                    self.state = 1;
                }
                None
            }
            1 => {
                if byte == FRAME_MAGIC_1 {
                    self.state = 2;
                } else if byte == FRAME_MAGIC_0 {
                    self.state = 1; // overlap: 0x55 0x55 0xAA
                } else {
                    self.state = 0;
                }
                None
            }
            2 => {
                self.ty = byte;
                self.crc = frame_crc_feed(0xFFFF, byte);
                self.state = 3;
                None
            }
            3 => {
                self.len = byte as usize;
                self.crc = frame_crc_feed(self.crc, byte);
                self.state = 4;
                None
            }
            4 => {
                self.len |= (byte as usize) << 8;
                self.crc = frame_crc_feed(self.crc, byte);
                if self.len > FRAME_MAX_PAYLOAD {
                    self.reset();
                    return None;
                }
                self.pos = 0;
                if self.buf.len() < self.len {
                    self.buf.resize(self.len, 0);
                }
                self.state = if self.len == 0 { 6 } else { 5 };
                None
            }
            5 => {
                self.buf[self.pos] = byte;
                self.pos += 1;
                self.crc = frame_crc_feed(self.crc, byte);
                if self.pos >= self.len {
                    self.state = 6;
                }
                None
            }
            6 => {
                self.crc_rx = byte as u16;
                self.state = 7;
                None
            }
            _ => {
                let rx = self.crc_rx | ((byte as u16) << 8);
                let (ty, len, ok) = (self.ty, self.len, rx == self.crc);
                if ok {
                    payload_out[..len].copy_from_slice(&self.buf[..len]);
                }
                self.reset();
                if ok { Some((ty, len)) } else { None }
            }
        }
    }

    fn reset(&mut self) {
        self.state = 0;
        self.ty = 0;
        self.len = 0;
        self.pos = 0;
        self.crc = 0xFFFF;
        self.crc_rx = 0;
    }
}

// Serial pose stream; reader thread owns the port, UI drains to latest.
// crossbeam (not std mpsc): the reader holds a cloned Receiver on the same
// shared queue so Full means one try_recv discard, then retry — oldest-drop,
// bounded, never blocks, never grows.
#[derive(Resource)]
struct SerialLink {
    pose_rx: crossbeam_channel::Receiver<Pose>,
}

// Video sender queue to the puck panel over the same CDC port. The render
// thread only copies the readback and enqueues; encoding, fragmentation and
// pacing happen on the sender thread. frame_tx is 2-deep and the sender holds a
// cloned Receiver, so a Full queue means one try_recv discards the older
// (already stale) frame and the newest still gets through: latency stays under
// one frame instead of dropping the newest.
#[derive(Resource)]
struct VideoTx {
    frame_tx: crossbeam_channel::Sender<Frame>,
    discard_rx: crossbeam_channel::Receiver<Frame>,
    free_tx: crossbeam_channel::Sender<Box<[u8]>>,
    free_rx: crossbeam_channel::Receiver<Box<[u8]>>,
    // Frames the render thread had to abandon outright (the queue was still
    // full after discarding the oldest). Superseded frames are the pacing
    // mechanism, not a fault, so they are not counted here.
    dropped: Arc<AtomicU32>,
    prev: Box<[u8]>, // previous readback, for the static-scene test
    prev_valid: bool,
    frame_id: u16,
}

pub struct TrackPlugin;

impl Plugin for TrackPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_scene);
        app.add_systems(Update, update);
        app.add_observer(on_video_readback);
    }
}

// Native-USB puck port out of the enumerated ports; None if it is unplugged.
fn pick_puck_port(ports: &[serialport::SerialPortInfo]) -> Option<&str> {
    ports
        .iter()
        .find(|p| {
            matches!(&p.port_type,
                serialport::SerialPortType::UsbPort(u) if u.vid == PUCK_VID && u.pid == PUCK_PID)
        })
        .map(|p| p.port_name.as_str())
}

fn open_puck_port() -> Box<dyn serialport::SerialPort> {
    let name = match std::env::var("PUCK_SERIAL") {
        Ok(p) => p,
        Err(_) => {
            let ports = serialport::available_ports().unwrap_or_default();
            let found = match pick_puck_port(&ports) {
                Some(p) => p.to_owned(),
                None => {
                    let seen: Vec<String> = ports
                        .iter()
                        .map(|p| format!("{} ({:?})", p.port_name, p.port_type))
                        .collect();
                    panic!(
                        "no puck on native USB {PUCK_VID:04x}:{PUCK_PID:04x}; enumerated {seen:?} \
                         — set PUCK_SERIAL=<port> to override"
                    );
                }
            };
            found
        }
    };
    println!("puck serial: {name} @ {SERIAL_BAUD}");
    serialport::new(&name, SERIAL_BAUD)
        .timeout(Duration::from_millis(50))
        .open()
        .unwrap_or_else(|e| panic!("open serial {name} (PUCK_SERIAL={name}): {e:?}"))
}

#[allow(dead_code)] // stamp asserted by tests; live path keys off seq only
struct Pose {
    seq: u32,
    stamp: u32,
    q: Quat,
    gyro: Vec3,
}

#[inline]
fn f(raw: &[u8], i: usize) -> f32 {
    f32::from_le_bytes([raw[i], raw[i + 1], raw[i + 2], raw[i + 3]])
}

fn parse_pose(raw: &[u8]) -> Pose {
    Pose {
        seq: u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]),
        stamp: u32::from_le_bytes([raw[4], raw[5], raw[6], raw[7]]),
        q: Quat::from_xyzw(f(raw, 8), f(raw, 12), f(raw, 16), f(raw, 20)),
        // Body-frame gyro rad/s.
        gyro: Vec3::new(f(raw, 24), f(raw, 28), f(raw, 32)),
    }
}

fn update(
    camera: Single<&mut Transform, With<LookRig>>,
    link: Res<SerialLink>,
    mut last_seq: Local<Option<u32>>,
    mut last_applied: Local<Option<Quat>>,
    time: Res<Time>,
) {
    // Drain to latest: only the newest pose is displayed, so everything but
    // the last queued pose is decoded-and-dropped.
    let latest: Option<Pose> = link.pose_rx.try_iter().last();
    // Readback capture: one persistent Readback entity lives for the whole
    // run (spawned once in setup_scene). It fires every frame on its own;
    // nothing is spawned or despawned per tick, so no entity churn.
    let Some(pose) = latest else {
        return;
    };

    if let Some(prev) = *last_seq {
        // Drain-to-latest normally skips ~4 packets/frame at 250Hz/60fps — not
        // loss; threshold 25 (~100ms) flags real bursts only.
        let lost = pose.seq.wrapping_sub(prev.wrapping_add(1));
        if lost > 25 {
            println!("seq gap: prev {prev} got {} (lost {lost})", pose.seq);
        }
    }
    *last_seq = Some(pose.seq);

    // Predict forward by PREDICT_TAU_S in the body frame (right-multiply).
    let ang = pose.gyro.length() * PREDICT_TAU_S;
    let unnorm = if ang > 1e-6 {
        pose.q * Quat::from_axis_angle(pose.gyro.normalize(), ang)
    } else {
        pose.q
    };
    let alpha = (1.0 - (-time.delta_secs() / SMOOTH_TAU_S).exp()).clamp(0.0, 1.0);
    let q_out = match *last_applied {
        Some(p) => p.slerp(unnorm, alpha),
        None => unnorm,
    }
    .normalize();

    camera.into_inner().rotation = q_out;
    *last_applied = Some(q_out);
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
) {
    // Static reference object so the camera rig's motion is visible.
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(2.9 / 2., 0.1 / 2., 5.7 / 2.))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.8))),
        Transform::from_xyz(0.0, 0.5, -6.0),
    ));
    commands.spawn((PointLight::default(), Transform::from_xyz(4.0, 8.0, 4.0)));

    // Off-screen 320x240 RGBA8 target: COPY_SRC added for the readback copy
    // (missing from new_target_texture's default usage).
    let mut target = Image::new_target_texture(
        VIDEO_W as u32,
        VIDEO_H as u32,
        TextureFormat::Rgba8Unorm,
        Some(TextureFormat::Rgba8UnormSrgb),
    );
    target.texture_descriptor.usage |= TextureUsages::COPY_SRC;
    let target = images.add(target);

    // The puck drives this rig; the capture camera rides along as a child, so
    // Bevy's transform propagation keeps it coincident (order -1 renders it
    // before the window camera). Rest quat (identity) looks down -Z, which is
    // where the reference object sits.
    commands
        .spawn((LookRig, Camera3d::default(), Transform::from_xyz(0.0, 1.5, 0.0)))
        .with_children(|rig| {
            rig.spawn((
                Camera3d::default(),
                Camera {
                    order: -1,
                    ..default()
                },
                RenderTarget::Image(target.clone().into()),
            ));
        });
    // Open the CDC port once: the original handle goes to the pose reader
    // thread, the clone to the paced video sender thread below.
    let port = open_puck_port();
    let writer = port.try_clone().unwrap_or_else(|e| panic!("clone serial port: {e:?}"));
    // Pose reader thread: blocking read, frame-parse, parse_pose, oldest-drop
    // send so a stalled UI never blocks the reader and never grows memory
    // (crossbeam receivers share one queue, so a cloned handle discards).
    let (pose_tx, pose_rx) = crossbeam_channel::bounded::<Pose>(512);
    let discard_rx = pose_rx.clone();
    thread::spawn(move || {
        let mut port = port;
        let mut scratch = [0u8; 1024];
        let mut parser = FrameParser::default();
        let mut payload = vec![0u8; FRAME_MAX_PAYLOAD];
        loop {
            let n = match port.read(&mut scratch) {
                Ok(n) => n,
                Err(_) => continue,
            };
            for &b in &scratch[..n] {
                let Some((ty, len)) = parser.push(b, &mut payload) else {
                    continue;
                };
                if ty != FRAME_POSE || len != PACKET_LEN {
                    continue;
                }
                let pose = parse_pose(&payload[..len]);
                if let Err(crossbeam_channel::TrySendError::Full(pose)) = pose_tx.try_send(pose) {
                    let _ = discard_rx.try_recv();
                    let _ = pose_tx.try_send(pose);
                }
            }
        }
    });
    commands.insert_resource(SerialLink { pose_rx });
    // JPEG sender thread: owns the cloned serial handle, encodes off the render
    // thread, fragments, and paces each fragment across the frame interval.
    // Fire-and-forget; write errors ignored. Returned buffers recycle through
    // free_tx so the steady state allocates nothing.
    let (frame_tx, frame_rx) = crossbeam_channel::bounded::<Frame>(2);
    let discard_rx = frame_rx.clone();
    let (free_tx, free_rx) = crossbeam_channel::unbounded::<Box<[u8]>>();
    let return_tx = free_tx.clone();
    let dropped = Arc::new(AtomicU32::new(0));
    let dropped_tx = dropped.clone();
    thread::spawn(move || {
        let mut writer = writer;
        let mut out: Vec<u8> = Vec::with_capacity(VIDEO_JPEG_MAX);
        let mut cached: Vec<u8> = Vec::new(); // last frame sent, for the heartbeat
        let mut cached_id = 0u16;
        let mut last_sent = Instant::now();
        let interval = Duration::from_micros(1_000_000 / VIDEO_TARGET_FPS);
        let mut log_at = Instant::now();
        let (mut sent, mut big, mut enc_us, mut enc_n) = (0u32, 0u32, 0u64, 0u32);
        loop {
            match frame_rx.recv_timeout(Duration::from_millis(50)) {
                Ok(frm) => {
                    let frame_start = Instant::now();
                    out.clear();
                    let mut enc = jpeg_encoder::Encoder::new(&mut out, JPEG_QUALITY);
                    enc.set_sampling_factor(jpeg_encoder::SamplingFactor::F_2_2); // 4:2:0
                    let encoded = enc.encode(
                        &frm.rgba,
                        VIDEO_W as u16,
                        VIDEO_H as u16,
                        jpeg_encoder::ColorType::Rgba,
                    );
                    enc_us += frame_start.elapsed().as_micros() as u64;
                    enc_n += 1;
                    let _ = return_tx.send(frm.rgba);
                    // `big`: frames skipped rather than truncated — an encode
                    // error, or more bytes than the device will assemble.
                    if encoded.is_err() || out.len() > VIDEO_JPEG_MAX {
                        big += 1;
                    } else {
                        cached.clear();
                        cached.extend_from_slice(&out);
                        cached_id = frm.id;
                        send_jpeg(&mut writer, frm.id, &out, frame_start, interval);
                        sent += 1;
                        last_sent = Instant::now();
                    }
                }
                Err(crossbeam_channel::RecvTimeoutError::Timeout) => {
                    // Static scene: the render thread enqueues nothing, so keep
                    // the device's idle RGB cycle away with a slow re-send.
                    if !cached.is_empty()
                        && last_sent.elapsed() >= Duration::from_millis(VIDEO_HEARTBEAT_MS)
                    {
                        send_jpeg(&mut writer, cached_id, &cached, Instant::now(), interval);
                        last_sent = Instant::now();
                    }
                }
                Err(crossbeam_channel::RecvTimeoutError::Disconnected) => break,
            }
            if log_at.elapsed() >= Duration::from_secs(1) {
                let enc_ms = if enc_n == 0 {
                    0.0
                } else {
                    enc_us as f64 / enc_n as f64 / 1000.0
                };
                println!(
                    "video {sent} fps drop {} big {big} enc {enc_ms:.1} ms",
                    dropped_tx.load(Ordering::Relaxed)
                );
                sent = 0;
                big = 0;
                enc_us = 0;
                enc_n = 0;
                log_at = Instant::now();
            }
        }
    });

    commands.insert_resource(VideoTx {
        frame_tx,
        discard_rx,
        free_tx,
        free_rx,
        dropped,
        prev: vec![0u8; VIDEO_W * 4 * VIDEO_H].into_boxed_slice(),
        prev_valid: false,
        frame_id: 0,
    });
    // One persistent readback: the component fires every frame on its own
    // until removed, so spawn once and never despawn — zero entity churn.
    commands.spawn(Readback::texture(target));
}

// Readback fires when the requested texture copy completes. Copy it into a
// pooled buffer and hand it to the sender thread; an unchanged frame enqueues
// nothing at all (the sender's heartbeat covers the static scene).
fn on_video_readback(readback: On<ReadbackComplete>, mut video: ResMut<VideoTx>) {
    let data = &readback.data;
    // Tight-pack invariant: 320*4 = 1280 is a multiple of the 256-byte row
    // alignment, so the readback is exactly 307200 bytes with no stride
    // padding. Fails loudly in debug/test if the GPU layout ever changes;
    // the release early-return below keeps the live path from slicing OOB.
    debug_assert_eq!(data.len(), VIDEO_W * 4 * VIDEO_H, "readback stride changed");
    if data.len() < VIDEO_W * 4 * VIDEO_H {
        return;
    }
    let src = &data[..VIDEO_W * 4 * VIDEO_H];
    if video.prev_valid && &video.prev[..] == src {
        return; // static scene: no buffer taken, nothing enqueued
    }
    let mut buf = video
        .free_rx
        .try_recv()
        .unwrap_or_else(|_| vec![0u8; VIDEO_W * 4 * VIDEO_H].into_boxed_slice());
    buf.copy_from_slice(src);
    video.prev.copy_from_slice(src);
    video.prev_valid = true;
    let frame = video.frame_id;
    video.frame_id = frame.wrapping_add(1);
    match video.frame_tx.try_send(Frame { rgba: buf, id: frame }) {
        Ok(()) => {}
        // Sender behind: supersede the queued frame — it is already stale by the
        // time the sender would reach it — and retry, so the newest frame wins
        // and latency stays under one frame.
        Err(crossbeam_channel::TrySendError::Full(frm)) => {
            if let Ok(old) = video.discard_rx.try_recv() {
                let _ = video.free_tx.send(old.rgba); // recycle: never grow the pool
            }
            if let Err(crossbeam_channel::TrySendError::Full(frm)) = video.frame_tx.try_send(frm) {
                video.dropped.fetch_add(1, Ordering::Relaxed);
                let _ = video.free_tx.send(frm.rgba);
            }
        }
        // Sender gone (shutdown): return the buffer to the pool.
        Err(crossbeam_channel::TrySendError::Disconnected(frm)) => {
            let _ = video.free_tx.send(frm.rgba);
        }
    }
    // Persistent readback entity stays alive — no despawn, no respawn.
}

#[cfg(test)]
mod tests {
    use super::*;

    // Shared byte-level fixture: the committed tests/packet_fixture.txt is the
    // single source of truth for the v2 wire layout, also asserted by the C++
    // producer test (tests/packet_test.cpp) against the same bytes.
    const FIXTURE: &str = include_str!("../../tests/packet_fixture.txt");

    fn fixture_bytes() -> Vec<u8> {
        let hex: String = FIXTURE
            .lines()
            .find(|l| !l.trim().is_empty() && !l.trim_start().starts_with('#'))
            .unwrap()
            .chars()
            .filter(|c| c.is_ascii_hexdigit())
            .collect();
        hex.as_bytes()
            .chunks(2)
            .map(|c| u8::from_str_radix(std::str::from_utf8(c).unwrap(), 16).unwrap())
            .collect()
    }

    #[test]
    fn pick_puck_port_prefers_native_usb_over_the_bridge() {
        let usb = |name: &str, vid: u16, pid: u16| serialport::SerialPortInfo {
            port_name: name.to_owned(),
            port_type: serialport::SerialPortType::UsbPort(serialport::UsbPortInfo {
                vid,
                pid,
                serial_number: None,
                manufacturer: None,
                product: None,
            }),
        };
        let bridge = usb("/dev/ttyACM1", 0x1a86, 0x55d3);
        let native = usb("/dev/ttyACM2", PUCK_VID, PUCK_PID);
        // Enumeration order must not decide the pick.
        assert_eq!(
            pick_puck_port(&[bridge.clone(), native.clone()]),
            Some("/dev/ttyACM2")
        );
        assert_eq!(pick_puck_port(&[native, bridge]), Some("/dev/ttyACM2"));
        assert_eq!(pick_puck_port(&[]), None);
    }

    #[test]
    fn parse_pose_roundtrips_the_committed_fixture() {
        let bytes = fixture_bytes();
        assert_eq!(bytes.len(), PACKET_LEN);
        let p = parse_pose(&bytes);
        assert_eq!((p.seq, p.stamp), (42, 1000));
        assert_eq!(p.q, Quat::from_xyzw(0.0, 0.0, 0.0, 1.0));
        let e = p.gyro - Vec3::new(0.5, -0.25, 1.5);
        assert!(e.length() < 1e-6, "gyro mismatch: {:?}", p.gyro);
    }
    #[test]
    fn frame_parser_recovers_garbage_prefixed_pose_and_rejects_bad_crc() {
        let payload = fixture_bytes();
        assert_eq!(payload.len(), PACKET_LEN);
        let mut frame = vec![0u8; 7 + PACKET_LEN];
        let n = build_frame(&mut frame, FRAME_POSE, &payload);
        assert_eq!(n, frame.len());
        // Garbage-prefixed valid frame recovers exactly.
        let mut parser = FrameParser::default();
        let mut out = vec![0u8; FRAME_MAX_PAYLOAD];
        let mut got = None;
        for &b in [0x00u8].iter().chain(frame.iter()) {
            if let Some((ty, len)) = parser.push(b, &mut out) {
                got = Some((ty, len));
            }
        }
        let (ty, len) = got.expect("valid frame must parse");
        assert_eq!((ty, len), (FRAME_POSE, PACKET_LEN));
        assert_eq!(&out[..len], &payload[..]);
        // One flipped payload byte: parser must yield nothing.
        let mut bad = frame.clone();
        bad[10] ^= 0x01;
        let mut parser = FrameParser::default();
        let mut yielded = false;
        for &b in &bad {
            if parser.push(b, &mut out).is_some() {
                yielded = true;
            }
        }
        assert!(!yielded, "corrupted frame must be rejected");
    }

    // Wire layout of one video fragment, pinned to the same literal bytes the
    // device-side test (tests/video_test.cpp) asserts: magic, FRAME_VIDEO,
    // len, 8 B header (frame/frag/count/len, all LE), payload, CRC-16.
    #[test]
    fn video_fragment_wire_layout_is_pinned() {
        let mut payload = Vec::new();
        payload.extend_from_slice(&0x1234u16.to_le_bytes()); // frame
        payload.extend_from_slice(&0u16.to_le_bytes()); // frag
        payload.extend_from_slice(&2u16.to_le_bytes()); // count
        payload.extend_from_slice(&3u16.to_le_bytes()); // len
        payload.extend_from_slice(&[0xAA, 0xBB, 0xCC]);
        let mut frame = vec![0u8; 7 + payload.len()];
        let n = build_frame(&mut frame, FRAME_VIDEO, &payload);
        let mut want = vec![
            0x55, 0xAA, 0x02, 0x0B, 0x00, // magic, FRAME_VIDEO, len = 11
            0x34, 0x12, 0x00, 0x00, 0x02, 0x00, 0x03, 0x00, // frame/frag/count/len
            0xAA, 0xBB, 0xCC, // fragment payload
        ];
        let mut crc: u16 = 0xFFFF;
        for &b in &frame[2..5 + payload.len()] {
            crc = frame_crc_feed(crc, b);
        }
        want.extend_from_slice(&crc.to_le_bytes());
        assert_eq!(&frame[..n], &want[..]);
    }

    // Fragments come out in order with the expected lengths, and the parser puts
    // them back together byte for byte (the device's assembler mirrors this).
    #[test]
    fn send_jpeg_fragments_and_the_parser_reassembles_them() {
        let blob: Vec<u8> = (0..VIDEO_FRAG_MAX * 2 + 17).map(|i| (i * 7) as u8).collect();
        let mut wire = Vec::new();
        // Duration::ZERO: nothing to pace against in a test.
        send_jpeg(&mut wire, 9, &blob, Instant::now(), Duration::ZERO);
        let mut parser = FrameParser::default();
        let mut out = vec![0u8; FRAME_MAX_PAYLOAD];
        let mut hdrs: Vec<(u16, u16, u16, u16)> = Vec::new();
        let mut got: Vec<u8> = Vec::new();
        for &b in &wire {
            let Some((ty, len)) = parser.push(b, &mut out) else {
                continue;
            };
            assert_eq!(ty, FRAME_VIDEO);
            let h = &out[..len];
            hdrs.push((
                u16::from_le_bytes([h[0], h[1]]),
                u16::from_le_bytes([h[2], h[3]]),
                u16::from_le_bytes([h[4], h[5]]),
                u16::from_le_bytes([h[6], h[7]]),
            ));
            got.extend_from_slice(&h[8..]);
        }
        assert_eq!(hdrs, vec![(9, 0, 3, 1280), (9, 1, 3, 1280), (9, 2, 3, 17)]);
        assert_eq!(got, blob);
    }
}
