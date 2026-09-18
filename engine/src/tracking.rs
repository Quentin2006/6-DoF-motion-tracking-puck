use std::io::{Read as _, Write as _};
use std::sync::atomic::{AtomicU64, Ordering};
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
const FRAME_VIDEO_RLE: u8 = 0x03;
const FRAME_MAX_PAYLOAD: usize = 1288;

// Per-chunk RLE over the 640 u16 LE words of one chunk (must mirror
// rle_decode_565 in device/video.h). Bit7 = 1 run (count 1..128 + one LE
// word), bit7 = 0 literal (count 1..128 + count LE words); a zero count
// nibble encodes 128. Worst case (alternating words) is ~1310 B, so the
// caller falls back to raw whenever the encoding is not strictly smaller.
fn rle_encode_565(chunk: &[u8]) -> Vec<u8> {
    debug_assert_eq!(chunk.len(), VIDEO_PAYLOAD_MAX);
    let words = chunk.len() / 2;
    let mut out = Vec::with_capacity(chunk.len());
    let word = |i: usize| u16::from_le_bytes([chunk[2 * i], chunk[2 * i + 1]]);
    let mut i = 0;
    while i < words {
        // Run length at i (cap 128: 0 encodes 128).
        let v = word(i);
        let mut run = 1;
        while i + run < words && run < 128 && word(i + run) == v {
            run += 1;
        }
        if run >= 3 {
            out.push(0x80 | ((run % 128) as u8));
            out.extend_from_slice(&v.to_le_bytes());
            i += run;
            continue;
        }
        // Literal run: extend until a 3+ run starts (cap 128).
        let start = i;
        let mut lit = 0;
        while i + lit < words && lit < 128 {
            let j = i + lit;
            let w = word(j);
            let mut r = 1;
            while j + r < words && r < 128 && word(j + r) == w {
                r += 1;
            }
            if r >= 3 && lit > 0 {
                break;
            }
            if r >= 3 {
                // A 3+ run starts immediately: emit as run, not literal.
                break;
            }
            lit += 1;
        }
        // Degenerate: single word before a run — emit as 1-literal.
        if lit == 0 {
            lit = 1;
        }
        out.push((lit % 128) as u8);
        for k in 0..lit {
            let w = word(start + k);
            out.extend_from_slice(&w.to_le_bytes());
        }
        i = start + lit;
    }
    out
}
// Transport+render delay extrapolated from packet gyro, and display smoothing.
const PREDICT_TAU_S: f32 = 0.008;
const SMOOTH_TAU_S: f32 = 0.006;

// Puck serial link: 1a86:55d3 bridge @ 921600 (real UART: baud is wire speed).
// ttyACM* number is unstable; the by-id symlink (serial-number based) is stable.
const DEFAULT_SERIAL_PORT: &str = "/dev/ttyACM0";
const SERIAL_BAUD: u32 = 921600;

// Stable open path: explicit PUCK_SERIAL wins, else first matching by-id node
// (1a86 bridge today, Espressif native-CDC if the transport moves), else the
// DEFAULT_SERIAL_PORT fallback above.
fn puck_port_name() -> String {
    if let Ok(name) = std::env::var("PUCK_SERIAL") {
        if !name.is_empty() {
            return name;
        }
    }
    if let Ok(dir) = std::fs::read_dir("/dev/serial/by-id") {
        let mut fallback: Option<String> = None;
        for entry in dir.flatten() {
            let n = entry.file_name().to_string_lossy().into_owned();
            let is_bridge = n.starts_with("usb-1a86_USB_Single_Serial_");
            let is_native = n.starts_with("usb-Espressif_USB_JTAG_serial_debug_unit_");
            if is_bridge {
                return format!("/dev/serial/by-id/{n}");
            }
            if is_native {
                fallback = Some(format!("/dev/serial/by-id/{n}"));
            }
        }
        if let Some(p) = fallback {
            return p;
        }
    }
    DEFAULT_SERIAL_PORT.into()
}

// Video stream to the puck panel: 320x240 RGB565 as FRAME_VIDEO frames.
// Must match device/video.h VIDEO_* constants.
const VIDEO_W: usize = 320;
const VIDEO_H: usize = 240;
const VIDEO_PAYLOAD_MAX: usize = 1280; // 2 rows x 320 px x 2 B
const VIDEO_CHUNKS: usize = 120; // 240 rows / 2 rows per 1280 B chunk
const VIDEO_FRAME_BYTES: usize = VIDEO_W * VIDEO_H * 2; // 153600 RGB565

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

// Encode one chunk into `frame_buf`, returning the frame bytes used (0
// on encode failure, never). Raw fallback when incompressible (the RLE worst
// case exceeds the raw size, so the comparison is strict).
fn encode_chunk(frame_buf: &mut [u8], id: u16, chunk: usize, bytes: &[u8]) -> usize {
    let enc = rle_encode_565(bytes);
    let (ty, body): (u8, &[u8]) = if enc.len() < VIDEO_PAYLOAD_MAX {
        (FRAME_VIDEO_RLE, &enc)
    } else {
        (FRAME_VIDEO, bytes)
    };
    let mut payload = [0u8; 8 + VIDEO_PAYLOAD_MAX];
    payload[0..2].copy_from_slice(&id.to_le_bytes());
    payload[2..4].copy_from_slice(&(chunk as u16).to_le_bytes());
    payload[4..6].copy_from_slice(&(VIDEO_CHUNKS as u16).to_le_bytes());
    payload[6..8].copy_from_slice(&(body.len() as u16).to_le_bytes());
    payload[8..8 + body.len()].copy_from_slice(body);
    build_frame(frame_buf, ty, &payload[..8 + body.len()])
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

// Step 2 throughput stats: worker threads bump atomics, the 1 Hz logger in
// `update` samples + resets. No per-packet logging (would perturb the read).
// Shared via Arc: the Resource holds one clone, worker threads hold clones.
#[derive(Resource, Default, Clone)]
struct LinkStats {
    inner: std::sync::Arc<LinkCounters>,
}

#[derive(Default)]
struct LinkCounters {
    poses: AtomicU64,        // reader: FRAME_POSE parsed
    reader_drops: AtomicU64, // reader: oldest-drop discards on Full
    readbacks: AtomicU64,    // render: on_video_readback calls
    offered: AtomicU64,      // render: frames offered to frame_tx
    dropped_full: AtomicU64, // render: frames returned to pool (queue full)
    frames_sent: AtomicU64,  // sender: frames drained from frame_rx
    dirty_sum: AtomicU64,    // sender: dirty chunks total
    dirty_max: AtomicU64,    // sender: max dirty chunks in one frame
    video_bytes: AtomicU64,  // sender: wire bytes via send_chunk
    write_us_sum: AtomicU64, // sender: write_all span total (us)
    write_us_max: AtomicU64, // sender: write_all span max (us)
    gap_events: AtomicU64,   // update: seq gaps > 25
    gap_lost: AtomicU64,     // update: poses skipped inside those gaps
}

// Video sender queue to the puck panel over the same CDC port.
// The render thread offers at most ~1 fps (1000 ms gate); the sender thread
// paces the full 120-chunk frame (idx 0..119 ascending, newest frame wins)
// at ~5 ms per chunk — the device barrier drops bursts, so no burst.
// frame_tx is 2-deep so the render thread never blocks
#[derive(Resource)]
struct VideoTx {
    frame_tx: SyncSender<Frame>,
    free_tx: crossbeam_channel::Sender<Box<[u8]>>,
    free_rx: crossbeam_channel::Receiver<Box<[u8]>>,
    frame_id: u16,
    last_accept: Option<Instant>,
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
    let name = puck_port_name();
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
    stats: Res<LinkStats>,
    mut last_seq: Local<Option<u32>>,
    mut last_applied: Local<Option<Quat>>,
    mut last_log: Local<Option<Instant>>,
    time: Res<Time>,
) {
    // Drain to latest: only the newest pose is displayed, so everything but
    // the last queued pose is decoded-and-dropped.
    let mut latest: Option<Pose> = None;
    for pose in link.pose_rx.try_iter() {
        latest = Some(pose);
    }
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
            stats.inner.gap_events.fetch_add(1, Ordering::Relaxed);
            stats.inner.gap_lost.fetch_add(lost as u64, Ordering::Relaxed);
        }
    }
    *last_seq = Some(pose.seq);
    // 1 Hz Step 2 line (stderr): pose/video throughput since last print.
    let now = Instant::now();
    let due = match *last_log {
        Some(t) => now.duration_since(t) >= Duration::from_secs(1),
        None => true,
    };
    if due {
        *last_log = Some(now);
        let load = |a: &AtomicU64| a.swap(0, Ordering::Relaxed);
        let c = &stats.inner;
        let poses = load(&c.poses);
        let rdrops = load(&c.reader_drops);
        let gaps = load(&c.gap_events);
        let lost_sum = load(&c.gap_lost);
        let readbacks = load(&c.readbacks);
        let offered = load(&c.offered);
        let dropped_full = load(&c.dropped_full);
        let frames = load(&c.frames_sent);
        let dirty_sum = load(&c.dirty_sum);
        let dirty_max = load(&c.dirty_max);
        let vid_b = load(&c.video_bytes);
        let wr_sum = load(&c.write_us_sum);
        let wr_max = load(&c.write_us_max);
        let dirty_avg = if frames > 0 { dirty_sum / frames } else { 0 };
        let wr_avg = if frames > 0 { wr_sum / frames } else { 0 };
        eprintln!(
            "link poseFps={poses} rdrop={rdrops} gaps={gaps} lost={lost_sum} readbackFps={readbacks} offered={offered} droppedFull={dropped_full} frames={frames} dirtyAvg={dirty_avg} dirtyMax={dirty_max} vidKB={} wrAvgUs={wr_avg} wrMaxUs={wr_max}",
            vid_b / 1024,
        );
    }

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
    // thread, the clone to the video sender thread below.
    let port = open_puck_port();
    let writer = port.try_clone().unwrap_or_else(|e| panic!("clone serial port: {e:?}"));
    let stats = LinkStats::default();
    commands.insert_resource(stats.clone());
    let reader_stats = stats.clone();
    let sender_stats = stats.clone();
    // Pose reader thread: blocking read, frame-parse, parse_pose, oldest-drop
    // send so a stalled UI never blocks the reader and never grows memory
    // (crossbeam receivers share one queue, so a cloned handle discards).
    let (pose_tx, pose_rx) = crossbeam_channel::bounded::<Pose>(512);
    let discard_rx = pose_rx.clone();
    thread::spawn(move || {
        let stats = reader_stats;
        let mut port = port;
        let mut scratch = [0u8; 1024];
        let mut parser = FrameParser::default();
        let mut payload = vec![0u8; FRAME_MAX_PAYLOAD];
        loop {
            let n = match port.read(&mut scratch) {
                Ok(0) => continue,
                Ok(n) => n,
                Err(_) => continue,
            };
            // Device 1 Hz status/log lines share the wire unframed; surface
            // full lines on stderr so `q ...` lines stay correlatable. The
            // 1 Hz report is ~200 B, so carry a small spill across reads
            // (a 1 KB scratch can split it). Thread-local: reader only.
            {
                thread_local! {
                    static SPILL: std::cell::RefCell<Vec<u8>> = std::cell::RefCell::new(Vec::new());
                }
                SPILL.with(|spill| {
                    let mut spill = spill.borrow_mut();
                    spill.extend_from_slice(&scratch[..n]);
                    let mut line_start = 0;
                    for i in 0..spill.len() {
                        if spill[i] == b'\n' {
                            // Binary pose frames share the wire and contain
                            // 0x0A, so a naive line_start..i segment is
                            // usually [binary tail + "q ..."]. Anchor on the
                            // last "q " in the segment and validate/print
                            // only that tail.
                            let seg = &spill[line_start..i];
                            let mut mark: Option<usize> = None;
                            let mut j = 0;
                            while j + 1 < seg.len() {
                                if seg[j] == b'q' && seg[j + 1] == b' ' {
                                    mark = Some(j);
                                }
                                j += 1;
                            }
                            if let Some(pos) = mark {
                                if let Ok(s) = std::str::from_utf8(&seg[pos..]) {
                                    let t = s.trim_end_matches('\r');
                                    if t.starts_with("q ")
                                        && t.bytes().all(|b| (0x20..=0x7e).contains(&b))
                                    {
                                        eprintln!("dev {t}");
                                    }
                                }
                            }
                            line_start = i + 1;
                        }
                    }
                    spill.drain(..line_start);
                    if spill.len() > 256 {
                        let drop = spill.len() - 256;
                        spill.drain(..drop);
                    }
                });
            }
            for &b in &scratch[..n] {
                let Some((ty, len)) = parser.push(b, &mut payload) else {
                    continue;
                };
                if ty != FRAME_POSE || len != PACKET_LEN {
                    continue;
                }
                let pose = parse_pose(&payload[..len]);
                stats.inner.poses.fetch_add(1, Ordering::Relaxed);
                if let Err(crossbeam_channel::TrySendError::Full(pose)) = pose_tx.try_send(pose) {
                    stats.inner.reader_drops.fetch_add(1, Ordering::Relaxed);
                    let _ = discard_rx.try_recv();
                    let _ = pose_tx.try_send(pose);
                }
            }
        }
    });
    commands.insert_resource(SerialLink { pose_rx });
    // Sender thread: owns the cloned serial handle, sends the whole
    // 320x240 frame (all 120 2-row chunks, idx ascending) every gated frame.
    // One write_all per chunk back-to-back (~10 fps on native CDC) — the
    // device per-row newest-wins buffer absorbs pacing. Fire-and-forget;
    // write errors ignored. Returned buffers recycle through free_tx so the
    // steady state allocates nothing.
    let (frame_tx, frame_rx) = sync_channel::<Frame>(2);
    let (free_tx, free_rx) = crossbeam_channel::unbounded::<Box<[u8]>>();
    let return_tx = free_tx.clone();
    thread::spawn(move || {
        let stats = sender_stats;
        let mut writer = writer;
        let mut scratch = [0u8; 7 + 8 + VIDEO_PAYLOAD_MAX];
        while let Ok(frm) = frame_rx.recv() {
            stats.inner.frames_sent.fetch_add(1, Ordering::Relaxed);
            let t0 = Instant::now();
            let mut bytes: u64 = 0;
            for chunk in 0..VIDEO_CHUNKS {
                let out = &frm.bytes[chunk * VIDEO_PAYLOAD_MAX..(chunk + 1) * VIDEO_PAYLOAD_MAX];
                let n = encode_chunk(&mut scratch, frm.id, chunk, out);
                if writer.write_all(&scratch[..n]).is_ok() {
                    bytes += n as u64;
                }
            }
            let us = t0.elapsed().as_micros().min(u128::from(u64::MAX)) as u64;
            stats.inner.dirty_sum.fetch_add(VIDEO_CHUNKS as u64, Ordering::Relaxed);
            stats.inner.video_bytes.fetch_add(bytes, Ordering::Relaxed);
            stats.inner.write_us_sum.fetch_add(us, Ordering::Relaxed);
            stats.inner.write_us_max.fetch_max(us, Ordering::Relaxed);
            let _ = return_tx.send(frm.bytes);
        }
    });

    commands.insert_resource(VideoTx {
        frame_tx,
        free_tx,
        free_rx,
        frame_id: 0,
        last_accept: None,
    });
    // One persistent readback: the component fires every frame on its own
    // until removed, so spawn once and never despawn — zero entity churn.
    commands.spawn(Readback::texture(target));
}

// Readback fires when the requested texture copy completes; pack RGB565 into a
// pooled buffer and hand it to the sender thread. Drop-if-behind: when
fn on_video_readback(readback: On<ReadbackComplete>, mut video: ResMut<VideoTx>, stats: Res<LinkStats>) {
    stats.inner.readbacks.fetch_add(1, Ordering::Relaxed);
    // Full-frame gate: at most ~10 fps (100 ms since last accepted frame;
    // native CDC has no UART budget, device drains ~8000 chunks/s). Excess
    // frames drop before the RGB565 pack — same pool return as the Full arm.
    let now = Instant::now();
    if let Some(t) = video.last_accept {
        if now.duration_since(t) < Duration::from_millis(100) {
            stats.inner.dropped_full.fetch_add(1, Ordering::Relaxed);
            return;
        }
    }
    video.last_accept = Some(now);
    let data = &readback.data;
    // Tight-pack invariant: 320*4 = 1280 is a multiple of the 256-byte row
    // alignment, so the readback is exactly 307200 bytes with no stride
    // padding. Fails loudly in debug/test if the GPU layout ever changes;
    // the release early-return below keeps the live path from slicing OOB.
    debug_assert_eq!(data.len(), VIDEO_W * 4 * VIDEO_H, "readback stride changed");
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
    stats.inner.offered.fetch_add(1, Ordering::Relaxed);
    match video.frame_tx.try_send(Frame { bytes: buf, id: frame }) {
        Ok(()) => {}
        Err(std::sync::mpsc::TrySendError::Full(frm)) => {
            // Sender behind: return the frame buffer to the pool instead of blocking.
            stats.inner.dropped_full.fetch_add(1, Ordering::Relaxed);
            let _ = video.free_tx.send(frm.bytes);
        }
        Err(std::sync::mpsc::TrySendError::Disconnected(frm)) => {
            stats.inner.dropped_full.fetch_add(1, Ordering::Relaxed);
            let _ = video.free_tx.send(frm.bytes);
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

    // Decode mirror of device/video.h rle_decode_565 (test-only): the same
    // scheme the sender's rle_encode_565 must invert.
    fn rle_decode_test(inp: &[u8]) -> Option<Vec<u16>> {
        let mut out = Vec::new();
        let mut ip = 0;
        while ip < inp.len() {
            let ctrl = inp[ip];
            ip += 1;
            let run = if (ctrl & 0x7F) == 0 { 128 } else { (ctrl & 0x7F) as usize };
            if ctrl & 0x80 != 0 {
                if ip + 2 > inp.len() {
                    return None;
                }
                let v = u16::from_le_bytes([inp[ip], inp[ip + 1]]);
                ip += 2;
                out.extend(std::iter::repeat(v).take(run));
            } else {
                if ip + run * 2 > inp.len() {
                    return None;
                }
                for _ in 0..run {
                    out.push(u16::from_le_bytes([inp[ip], inp[ip + 1]]));
                    ip += 2;
                }
            }
        }
        Some(out)
    }

    fn words_le(words: &[u16]) -> Vec<u8> {
        let mut b = Vec::with_capacity(words.len() * 2);
        for w in words {
            b.extend_from_slice(&w.to_le_bytes());
        }
        b
    }

    #[test]
    fn rle_roundtrips_runs_literals_and_boundaries() {
        // Solid chunk: one 640-word run compresses to 5 runs of 128.
        let solid = words_le(&vec![0xF800u16; 640]);
        let enc = rle_encode_565(&solid);
        // Exact wire bytes: five (0x80, 00 F8) triples — 0 count = 128.
        let mut expect = Vec::new();
        for _ in 0..5 {
            expect.extend_from_slice(&[0x80, 0x00, 0xF8]);
        }
        assert_eq!(enc, expect);
        let dec = rle_decode_test(&enc).expect("solid decodes");
        assert_eq!(dec, vec![0xF800u16; 640]);
        // Alternating words: no 3+ run, all literals, strictly larger.
        let mut alt = Vec::with_capacity(640);
        for i in 0..640 {
            alt.push(if i % 2 == 0 { 0x0000u16 } else { 0xFFFFu16 });
        }
        let enc_alt = rle_encode_565(&words_le(&alt));
        assert!(
            enc_alt.len() >= VIDEO_PAYLOAD_MAX,
            "incompressible must not shrink"
        );
        let dec_alt = rle_decode_test(&enc_alt).expect("alt decodes");
        assert_eq!(dec_alt, alt);
        // 128-word boundary: exactly 128 identical words = one run byte.
        let mut bnd = vec![0x1234u16; 128];
        bnd.extend((0..512).map(|i| i as u16));
        let enc_bnd = rle_encode_565(&words_le(&bnd));
        let dec_bnd = rle_decode_test(&enc_bnd).expect("boundary decodes");
        assert_eq!(dec_bnd, bnd);
    }

    #[test]
    fn pack_ppm_gradient_roundtrip() {
        // Synthetic 320x240 RGBA8 gradient through the exact pack expression
        // in on_video_readback; pins the RGB565 LE words at four corners.
        let mut rgba = vec![0u8; VIDEO_W * 4 * VIDEO_H];
        for y in 0..VIDEO_H {
            for x in 0..VIDEO_W {
                let o = (y * VIDEO_W + x) * 4;
                rgba[o] = (x & 255) as u8;
                rgba[o + 1] = (y & 255) as u8;
                rgba[o + 2] = ((x + y) & 255) as u8;
                rgba[o + 3] = 255;
            }
        }
        let mut out = vec![0u8; VIDEO_FRAME_BYTES];
        for (dst, px) in out
            .chunks_exact_mut(2)
            .zip(rgba[..VIDEO_W * 4 * VIDEO_H].chunks_exact(4))
        {
            let v: u16 =
                ((px[0] as u16 >> 3) << 11) | ((px[1] as u16 >> 2) << 5) | (px[2] as u16 >> 3);
            dst.copy_from_slice(&v.to_le_bytes());
        }
        assert_eq!(out.len(), VIDEO_FRAME_BYTES);
        let word = |x: usize, y: usize| {
            let o = (y * VIDEO_W + x) * 2;
            u16::from_le_bytes([out[o], out[o + 1]])
        };
        assert_eq!(word(0, 0), 0x0000);
        assert_eq!(word(319, 0), 0x3807);
        assert_eq!(word(0, 239), 0x077D);
        assert_eq!(word(319, 239), 0x3F65);
    }
}
