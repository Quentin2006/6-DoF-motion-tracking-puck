use std::io::{Read as _, Write as _};
use std::sync::mpsc::{SyncSender, sync_channel};
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
const FRAME_VIDEO: u8 = 0x02;
const FRAME_MAX_PAYLOAD: usize = 1288;

// Transport+render delay extrapolated from packet gyro, and display smoothing.
const PREDICT_TAU_S: f32 = 0.008;
const SMOOTH_TAU_S: f32 = 0.006;

// Puck serial link: /dev/ttyACM0 @ 115200 nominal (native USB-CDC).
const DEFAULT_SERIAL_PORT: &str = "/dev/ttyACM0";
const SERIAL_BAUD: u32 = 115200;

// Video stream to the puck panel: 320x240 RGB565 as FRAME_VIDEO frames.
// Must match device/video.h VIDEO_* constants.
const VIDEO_W: usize = 320;
const VIDEO_H: usize = 240;
const VIDEO_PAYLOAD_MAX: usize = 1280; // 2 rows x 320 px x 2 B
const VIDEO_CHUNK_ROWS: usize = VIDEO_PAYLOAD_MAX / (VIDEO_W * 2); // 2
const VIDEO_CHUNKS: usize = VIDEO_H / VIDEO_CHUNK_ROWS; // 120
const VIDEO_FRAME_BYTES: usize = VIDEO_W * VIDEO_H * 2; // 153600 RGB565
const VIDEO_FRAME_PERIOD: Duration = Duration::from_millis(200); // 5 fps over USB-FS

struct Frame {
    bytes: Box<[u8]>,
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

// Video sender queue to the puck panel over the same CDC port.
// The sender thread owns a cloned serial handle and paces one chunk per
// ~1666 us; frame_tx is 2-deep so the render thread never blocks (drop-if-behind).
#[derive(Resource)]
struct VideoTx {
    frame_tx: SyncSender<Frame>,
    free_tx: crossbeam_channel::Sender<Box<[u8]>>,
    free_rx: crossbeam_channel::Receiver<Box<[u8]>>,
    target: Handle<Image>,
    last_request: Instant,
    pending: bool,
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

fn open_puck_port() -> Box<dyn serialport::SerialPort> {
    let name = std::env::var("PUCK_SERIAL").unwrap_or_else(|_| DEFAULT_SERIAL_PORT.into());
    serialport::new(&name, SERIAL_BAUD)
        .timeout(Duration::from_millis(50))
        .open()
        .unwrap_or_else(|e| panic!("open serial {name} (PUCK_SERIAL={name}): {e:?}"))
}


#[inline]
fn f(raw: &[u8], i: usize) -> f32 {
    f32::from_le_bytes([raw[i], raw[i + 1], raw[i + 2], raw[i + 3]])
}

#[inline]
fn header(raw: &[u8]) -> (u32, u32) {
    (
        u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]),
        u32::from_le_bytes([raw[4], raw[5], raw[6], raw[7]]),
    )
}

#[allow(dead_code)] // stamp asserted by tests; live path keys off seq only
struct Pose {
    seq: u32,
    stamp: u32,
    q: Quat,
    gyro: Vec3,
}

fn parse_pose(raw: &[u8]) -> Pose {
    let (seq, stamp) = header(raw);
    Pose {
        seq,
        stamp,
        q: Quat::from_xyzw(f(raw, 8), f(raw, 12), f(raw, 16), f(raw, 20)),
        // Body-frame gyro rad/s.
        gyro: Vec3::new(f(raw, 24), f(raw, 28), f(raw, 32)),
    }
}

fn update(
    mut commands: Commands,
    camera: Single<&mut Transform, With<LookRig>>,
    link: Res<SerialLink>,
    mut video: Option<ResMut<VideoTx>>,
    mut last_seq: Local<Option<u32>>,
    mut last_applied: Local<Option<Quat>>,
    time: Res<Time>,
) {
    // Drain to latest: only the newest pose is displayed, so everything but
    // the last queued pose is decoded-and-dropped.
    let mut latest: Option<Pose> = None;
    for pose in link.pose_rx.try_iter() {
        latest = Some(pose);
    }
    // Throttle the GPU readback: one request per frame period max (the
    // readback fires every frame while its component exists).
    if let Some(v) = video.as_mut() {
        let now = Instant::now();
        if !v.pending && now.duration_since(v.last_request) >= VIDEO_FRAME_PERIOD {
            commands.spawn(Readback::texture(v.target.clone()));
            v.pending = true;
            v.last_request = now;
        }
    }
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
    // Paced sender thread: owns the cloned serial handle, one chunk every
    // ~1666 us against absolute per-chunk deadlines (jitter never compounds).
    // Serial fire-and-forget; write errors ignored. Returned buffers recycle
    // through free_tx so the steady state allocates nothing.
    let (frame_tx, frame_rx) = sync_channel::<Frame>(2);
    let (free_tx, free_rx) = crossbeam_channel::unbounded::<Box<[u8]>>();
    let return_tx = free_tx.clone();
    thread::spawn(move || {
        let mut writer = writer;
        while let Ok(frm) = frame_rx.recv() {
            let t0 = Instant::now();
            for (chunk, out) in frm.bytes.chunks_exact(VIDEO_PAYLOAD_MAX).enumerate() {
                let deadline = t0 + VIDEO_FRAME_PERIOD / VIDEO_CHUNKS as u32 * (chunk as u32 + 1);
                let now = Instant::now();
                if deadline > now {
                    thread::sleep(deadline - now);
                }
                let mut payload = [0u8; 8 + VIDEO_PAYLOAD_MAX];
                payload[0..2].copy_from_slice(&frm.id.to_le_bytes());
                payload[2..4].copy_from_slice(&(chunk as u16).to_le_bytes());
                payload[4..6].copy_from_slice(&(VIDEO_CHUNKS as u16).to_le_bytes());
                payload[6..8].copy_from_slice(&(VIDEO_PAYLOAD_MAX as u16).to_le_bytes());
                payload[8..8 + out.len()].copy_from_slice(out);
                let mut frame = [0u8; 7 + 8 + VIDEO_PAYLOAD_MAX];
                let n = build_frame(&mut frame, FRAME_VIDEO, &payload[..8 + out.len()]);
                let _ = writer.write_all(&frame[..n]);
            }
            let _ = return_tx.send(frm.bytes);
        }
    });

    commands.insert_resource(VideoTx {
        frame_tx,
        free_tx,
        free_rx,
        target,
        // One period in the past so the first readback is requested immediately.
        last_request: Instant::now() - VIDEO_FRAME_PERIOD,
        pending: false,
        frame_id: 0,
    });
}

// Readback fires when the requested texture copy completes; pack RGB565 into a
// pooled buffer and hand it to the paced sender thread. Drop-if-behind: when
// the 2-deep queue is full the frame is returned to the pool instead of
// blocking the render thread.
fn on_video_readback(readback: On<ReadbackComplete>, mut video: ResMut<VideoTx>) {
    let data = &readback.data;
    // RGBA8, 4 B per pixel; stride == row length (VIDEO_W * 4 is 256-aligned).
    if data.len() < VIDEO_W * 4 * VIDEO_H {
        return;
    }
    let mut buf = video
        .free_rx
        .try_recv()
        .unwrap_or_else(|_| vec![0u8; VIDEO_FRAME_BYTES].into_boxed_slice());
    // One frame = 120 chunks x 1280 RGB565 bytes from 307200 RGBA8 bytes.
    for (dst, px) in buf
        .chunks_exact_mut(2)
        .zip(data[..VIDEO_W * 4 * VIDEO_H].chunks_exact(4))
    {
        let v: u16 = ((px[0] as u16 >> 3) << 11) | ((px[1] as u16 >> 2) << 5) | (px[2] as u16 >> 3);
        dst.copy_from_slice(&v.to_le_bytes());
    }
    let frame = video.frame_id;
    video.frame_id = frame.wrapping_add(1);
    match video.frame_tx.try_send(Frame { bytes: buf, id: frame }) {
        Ok(()) => {}
        Err(std::sync::mpsc::TrySendError::Full(frm)) => {
            // Sender behind: return the frame buffer to the pool instead of blocking.
            let _ = video.free_tx.send(frm.bytes);
        }
        Err(std::sync::mpsc::TrySendError::Disconnected(frm)) => {
            let _ = video.free_tx.send(frm.bytes);
        }
    }
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
}
