use std::io::{self, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use crate::usb_bulk::BulkLink;
use bevy::camera::{RenderTarget, Viewport};
use bevy::prelude::*;
use bevy::render::gpu_readback::{Readback, ReadbackComplete};
use bevy::render::render_resource::{TextureFormat, TextureUsages};

// Marker for the camera rig driven by the tracking puck (look-around).
#[derive(Component)]
struct LookRig;

// must match device/packet.h QUAT_PACKET_LEN_V2 (pose payload inside a FRAME_POSE frame)
const PACKET_LEN: usize = 36;

// Framed vendor-bulk contract, mirroring device/frame.h (not a shared crate).
const FRAME_MAGIC_0: u8 = 0x55;
const FRAME_MAGIC_1: u8 = 0xAA;
const FRAME_POSE: u8 = 0x01;
const FRAME_VIDEO: u8 = 0x02; // one blob fragment: 8 B header + <= 1280 B
const FRAME_VIDEO_RECT: u8 = 0x05; // the frame's dirty rectangles: 4 + 11*rects B
const FRAME_MAX_PAYLOAD: usize = 1288; // 8 + VIDEO_FRAG_MAX

// Transport+render delay extrapolated from packet gyro, and display smoothing.
const PREDICT_TAU_S: f32 = 0.008;
const SMOOTH_TAU_S: f32 = 0.006;

// Video stream to the puck: the stereo pair as one or two baseline JPEGs
// covering only the rectangles that changed, fragmented into FRAME_VIDEO
// frames and preceded by a FRAME_VIDEO_RECT frame with the rectangle table.
// Must match device/src/display/video.h VIDEO_* constants.
//
// Decoding the whole 640x240 canvas every frame costs the device ~17 ms of
// JPEGDEC (pixel-bound: measured, and quality does not move it), which capped
// the panel near 31 fps no matter how little of the frame differed. Sending
// only the dirty rectangles drops that to ~1 ms for a typical frame, leaving
// the dirty-area SPI blit as the cost.
const VIDEO_W: usize = 640;
const VIDEO_H: usize = 240;
const VIDEO_PANEL_W: usize = 320; // one panel's half of the canvas, i.e. one eye
const VIDEO_TILE: usize = 16; // tile grid == the 4:2:0 MCU the device decodes
const VIDEO_RECTS_MAX: usize = 4; // dirty bbox + repair quadrant, per eye
const VIDEO_RECT_LEN: usize = 11; // eye u8 | x,y,w,h u16 LE | jpeg_len u16 LE
const VIDEO_FRAG_MAX: usize = 1280; // JPEG bytes per fragment; 8 + this = FRAME_MAX_PAYLOAD
const VIDEO_JPEG_MAX: usize = 16384; // device assembly buffer, and the largest blob accepted
const JPEG_QUALITY: u8 = 75; // the one knob trading fidelity for frame rate
const VIDEO_TARGET_FPS: u64 = 40;
const VIDEO_HEARTBEAT_MS: u64 = 2000; // static scene: re-send so the device stays in video mode
// Re-send the whole canvas at least this often, changed or not. With rectangles
// the device holds no full-canvas pixels, so this is the only repair for panels
// it repainted itself (the idle RGB cycle) and for whatever a lost frame left
// stale.
//
// It is spread over KEYFRAME_STEPS frames, one slice per eye each: painting a
// whole canvas is 600 tiles of SPI (~102 ms measured), five times a 40 fps
// frame budget, and doing it in one frame made the panel hitch every two
// seconds. Rolled, each repair frame is one slice more than a normal one.
const VIDEO_KEYFRAME_MS: u64 = 2000;
const VIDEO_KEYFRAME_STEPS: usize = 4;

// One slice of a panel, for the repair pass: four vertical strips per eye, 5
// tiles wide and the full height. Full height rather than quadrants because the
// cost that matters is the blit: 75 tiles of SPI (~13 ms measured) plus that
// frame's dirty box fits inside a 40 fps frame, where a quadrant's 150 tiles did
// not (58 ms measured).
fn repair_slice(eye: u8, step: usize) -> Rect {
    let w = VIDEO_PANEL_W as u16 / VIDEO_KEYFRAME_STEPS as u16;
    Rect {
        eye,
        x: w * step as u16,
        y: 0,
        w,
        h: VIDEO_H as u16,
    }
}

// Eye separation, matched to the panel/lens spacing: the eyes sit at -IPD_M/2
// and +IPD_M/2. It is the only stereo-geometry knob — both eyes keep Bevy's
// default fov and get their 4:3 aspect from their own viewport.
const IPD_M: f32 = 0.064;

// One dirty rectangle of one eye, in eye-local coordinates, 16-px aligned on
// every side (a whole number of 4:2:0 MCUs, and of the device's tiles).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Rect {
    eye: u8,
    x: u16,
    y: u16,
    w: u16,
    h: u16,
}

struct Frame {
    rgba: Box<[u8]>, // VIDEO_W x VIDEO_H RGBA8 readback, pooled
    // Exactly what changed, per eye, computed here because the device has no
    // room for a shadow (two 320x240 RGB565 shadows are 307 KB against 82 KB
    // free) and no way to decode less than it is sent. Empty (rects_n == 0)
    // means the frame is identical to the previous one.
    rects: [Rect; VIDEO_RECTS_MAX],
    rects_n: usize,
    id: u16,
}

// Bounding box of the changed tiles of each eye, or both whole panels when
// `full`. A tile is dirty when any of its VIDEO_TILE rows differs by a byte:
// 600 tiles x 16 rows = 9600 memcmps of 64 B, ~1% of a core, against a decode
// and blit cost that is proportional to what it paints.
fn dirty_rects(cur: &[u8], prev: &[u8], full: bool) -> ([Rect; VIDEO_RECTS_MAX], usize) {
    const COLS: usize = VIDEO_PANEL_W / VIDEO_TILE;
    const ROWS: usize = VIDEO_H / VIDEO_TILE;
    let row_bytes = VIDEO_W * 4;
    let whole = Rect {
        eye: 0,
        x: 0,
        y: 0,
        w: VIDEO_PANEL_W as u16,
        h: VIDEO_H as u16,
    };
    let mut rects = [whole; VIDEO_RECTS_MAX];
    if full {
        rects[0].eye = 0;
        rects[1].eye = 1;
        return (rects, 2);
    }
    let mut n = 0;
    for eye in 0..2 {
        let (mut min_tx, mut max_tx) = (COLS, 0);
        let (mut min_ty, mut max_ty) = (ROWS, 0);
        for ty in 0..ROWS {
            for tx in 0..COLS {
                let x = (eye * VIDEO_PANEL_W + tx * VIDEO_TILE) * 4;
                let dirty = (0..VIDEO_TILE).any(|y| {
                    let off = (ty * VIDEO_TILE + y) * row_bytes + x;
                    cur[off..off + VIDEO_TILE * 4] != prev[off..off + VIDEO_TILE * 4]
                });
                if dirty {
                    min_tx = min_tx.min(tx);
                    max_tx = max_tx.max(tx);
                    min_ty = min_ty.min(ty);
                    max_ty = max_ty.max(ty);
                }
            }
        }
        if min_tx <= max_tx {
            rects[n] = Rect {
                eye: eye as u8,
                x: (min_tx * VIDEO_TILE) as u16,
                y: (min_ty * VIDEO_TILE) as u16,
                w: ((max_tx - min_tx + 1) * VIDEO_TILE) as u16,
                h: ((max_ty - min_ty + 1) * VIDEO_TILE) as u16,
            };
            n += 1;
        }
    }
    (rects, n)
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

// Send the rectangle table for one frame. It must land before the blob's
// fragments: the device paints the rectangles the table names, and drops a
// table it cannot paint (a rect past the panel edge, off the MCU grid, or
// whose lengths do not add up to the blob) rather than guess.
fn send_rects<W: Write>(writer: &mut W, id: u16, rects: &[Rect], lens: &[usize]) -> io::Result<()> {
    let n = rects.len();
    let mut payload = [0u8; 4 + VIDEO_RECT_LEN * VIDEO_RECTS_MAX];
    payload[0..2].copy_from_slice(&id.to_le_bytes());
    payload[2..4].copy_from_slice(&(n as u16).to_le_bytes());
    for (i, (r, len)) in rects.iter().zip(lens).enumerate() {
        let o = 4 + i * VIDEO_RECT_LEN;
        payload[o] = r.eye;
        payload[o + 1..o + 3].copy_from_slice(&r.x.to_le_bytes());
        payload[o + 3..o + 5].copy_from_slice(&r.y.to_le_bytes());
        payload[o + 5..o + 7].copy_from_slice(&r.w.to_le_bytes());
        payload[o + 7..o + 9].copy_from_slice(&r.h.to_le_bytes());
        payload[o + 9..o + 11].copy_from_slice(&(*len as u16).to_le_bytes());
    }
    let mut frame = [0u8; 7 + 4 + VIDEO_RECT_LEN * VIDEO_RECTS_MAX];
    let n = build_frame(
        &mut frame,
        FRAME_VIDEO_RECT,
        &payload[..4 + rects.len() * VIDEO_RECT_LEN],
    );
    writer.write_all(&frame[..n])
}

// Copy one rectangle out of the readback, RGBA rows packed tight, ready for the
// encoder. `out` must hold exactly w*h*4 bytes.
fn crop_rgba(out: &mut [u8], rgba: &[u8], r: &Rect) {
    let (w, h) = (r.w as usize, r.h as usize);
    let row_bytes = VIDEO_W * 4;
    let left = r.eye as usize * VIDEO_PANEL_W + r.x as usize; // canvas column
    for row in 0..h {
        let src = (r.y as usize + row) * row_bytes + left * 4;
        out[row * w * 4..(row + 1) * w * 4].copy_from_slice(&rgba[src..src + w * 4]);
    }
}

// Fragment the blob into FRAME_VIDEO frames and write them, spreading the
// fragments over the frame interval instead of bursting the whole image.
//
// Fragment size and pacing stay unchanged after the transport cutover: the bulk
// OUT endpoint still has a 64 B FIFO, and decode/SPI—not USB bandwidth—remain
// the measured panel-rate limit. Fragment i leaves at frame_start +
// i*interval/count, so the first piece goes out immediately and the last lands
// on the frame's grid point.
fn send_blob<W: Write>(
    writer: &mut W,
    id: u16,
    blob: &[u8],
    frame_start: Instant,
    interval: Duration,
) -> io::Result<()> {
    let count = blob.len().div_ceil(VIDEO_FRAG_MAX).max(1);
    let per_frag = interval / count as u32;
    let mut payload = [0u8; 8 + VIDEO_FRAG_MAX];
    let mut frame = [0u8; 7 + FRAME_MAX_PAYLOAD];
    for (i, part) in blob.chunks(VIDEO_FRAG_MAX).enumerate() {
        payload[0..2].copy_from_slice(&id.to_le_bytes());
        payload[2..4].copy_from_slice(&(i as u16).to_le_bytes());
        payload[4..6].copy_from_slice(&(count as u16).to_le_bytes());
        payload[6..8].copy_from_slice(&(part.len() as u16).to_le_bytes());
        payload[8..8 + part.len()].copy_from_slice(part);
        let n = build_frame(&mut frame, FRAME_VIDEO, &payload[..8 + part.len()]);
        writer.write_all(&frame[..n])?;
        let due = frame_start + per_frag * i as u32;
        if let Some(left) = due.checked_duration_since(Instant::now()) {
            thread::sleep(left);
        }
    }
    Ok(())
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
    // payload is in `payload_out[..len]`. Arbitrary bulk bytes are consumed
    // and ignored until the next valid frame.
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

// Bulk pose stream; reader thread owns bulk IN, UI drains to latest.
// crossbeam (not std mpsc): the reader holds a cloned Receiver on the same
// shared queue so Full means one try_recv discard, then retry — oldest-drop,
// bounded, never blocks, never grows.
#[derive(Resource)]
struct PuckLink {
    pose_rx: crossbeam_channel::Receiver<Pose>,
}

// Video sender queue to the puck panels over bulk OUT. The render thread only
// copies the readback and enqueues; encoding, fragmentation and pacing happen
// on the sender thread. frame_tx is 2-deep, and a full queue means this
// readback is skipped rather than superseding the queued baseline. `prev` may
// only advance to a frame the sender took; superseding instead would leave the
// difference between queued frames on the panels forever.
#[derive(Resource)]
struct VideoTx {
    frame_tx: crossbeam_channel::Sender<Frame>,
    free_tx: crossbeam_channel::Sender<Box<[u8]>>,
    free_rx: crossbeam_channel::Receiver<Box<[u8]>>,
    // Readbacks the sender was too busy to take: a load meter, not a fault —
    // they are the pipeline settling to the frame rate the links can carry.
    skipped: Arc<AtomicU32>,
    prev: Box<[u8]>, // last readback *enqueued* (i.e. what the panels will show)
    prev_valid: bool,
    full_at: Instant,     // when the last whole-canvas repair started
    keyframe_step: usize, // quadrant cursor of the repair pass in progress
    frame_id: u16,
    // The persistent readback entity and the target it copies. The component is
    // attached only while the sender can take a frame: a readback is a
    // full-frame GPU->CPU copy, and the app renders far faster than the link, so
    // asking for one every frame threw ~99% of that traffic away.
    readback_entity: Entity,
    target: Handle<Image>,
}

// Recenter key state: the puck orientation that the view should treat as the
// reference. Identity means "the device's own rest frame", which is not the
// reference view — see `update`.
#[derive(Resource)]
struct ViewBias(Quat);

impl Default for ViewBias {
    fn default() -> Self {
        Self(Quat::IDENTITY)
    }
}

pub struct TrackPlugin;

impl Plugin for TrackPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ViewBias>();
        app.add_systems(Startup, setup_scene);
        app.add_systems(Update, (update, throttle_readback));
        app.add_observer(on_video_readback);
    }
}

// Ask the GPU for a readback only when the sender can take one. The sender is
// the pacer, so the readback rate follows the link instead of the render rate;
// `skip` in the video log then only counts the rare race where the queue filled
// between the check and the send.
fn throttle_readback(
    mut commands: Commands,
    attached: Query<Entity, With<Readback>>,
    video: Res<VideoTx>,
) {
    match (video.frame_tx.is_full(), attached.iter().next()) {
        (true, Some(e)) => {
            commands.entity(e).remove::<Readback>();
        }
        (false, None) => {
            commands
                .entity(video.readback_entity)
                .insert(Readback::texture(video.target.clone()));
        }
        _ => {}
    }
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
    link: Res<PuckLink>,
    mut last_seq: Local<Option<u32>>,
    mut last_applied: Local<Option<Quat>>,
    keys: Res<ButtonInput<KeyCode>>,
    mut bias: ResMut<ViewBias>,
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
    // The device bakes a fixed ±90° X sandwich into the display quat, so the
    // puck's rest orientation is not the reference view; the bias is whatever
    // orientation the user calls "forward", captured on R.
    if keys.just_pressed(KeyCode::KeyR) {
        bias.0 = unnorm;
        // Drop the smoothing history: with no previous sample the first biased
        // pose is emitted as-is, landing on the reference in one frame rather
        // than slerping there over SMOOTH_TAU_S.
        *last_applied = None;
    }
    // Left-multiply the inverse: `a * b` applies b then a, so this re-zeros in
    // the world frame. Right-multiplying would offset in the puck's own body
    // frame, where the bias is meaningless once the device has turned.
    let biased = bias.0.conjugate() * unnorm;
    let alpha = (1.0 - (-time.delta_secs() / SMOOTH_TAU_S).exp()).clamp(0.0, 1.0);
    let q_out = match *last_applied {
        Some(p) => p.slerp(biased, alpha),
        None => biased,
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

    // Off-screen 640x240 RGBA8 target holding the stereo pair: COPY_SRC added
    // for the readback copy (missing from new_target_texture's default usage).
    let mut target = Image::new_target_texture(
        VIDEO_W as u32,
        VIDEO_H as u32,
        TextureFormat::Rgba8Unorm,
        Some(TextureFormat::Rgba8UnormSrgb),
    );
    target.texture_descriptor.usage |= TextureUsages::COPY_SRC;
    let target = images.add(target);

    // The puck drives this rig; the two eye cameras ride along as children, so
    // Bevy's transform propagation keeps them on the rig and IPD_M apart from
    // each other. Their viewports are the two halves of the target (order
    // -2/-1 render before the window camera), and each eye's projection is
    // built from its own 320x240 viewport — so both are 4:3 without an aspect
    // override. Rest quat (identity) looks down -Z, which is where the
    // reference object sits.
    commands
        .spawn((
            LookRig,
            Camera3d::default(),
            Transform::from_xyz(0.0, 1.5, 0.0),
        ))
        .with_children(|rig| {
            for i in 0..2u32 {
                rig.spawn((
                    Camera3d::default(),
                    Camera {
                        order: -2 + i as isize,
                        viewport: Some(Viewport {
                            physical_position: UVec2::new(i * VIDEO_PANEL_W as u32, 0),
                            physical_size: UVec2::new(VIDEO_PANEL_W as u32, VIDEO_H as u32),
                            depth: 0.0..1.0,
                        }),
                        ..default()
                    },
                    RenderTarget::Image(target.clone().into()),
                    // Eye 0 is the left half of the canvas at -IPD_M/2, eye 1
                    // the right at +IPD_M/2 (Bevy's +X is right when looking
                    // down -Z).
                    Transform::from_xyz((i as f32 - 0.5) * IPD_M, 0.0, 0.0),
                ));
            }
        });
    // Open and claim the vendor interface once. Arc-backed BulkLink clones let
    // the pose reader and video writer share that same handle in both directions.
    let link = BulkLink::open();
    let writer = link.clone();
    // Pose reader thread: blocking read, frame-parse, parse_pose, oldest-drop
    // send so a stalled UI never blocks the reader and never grows memory
    // (crossbeam receivers share one queue, so a cloned handle discards).
    let (pose_tx, pose_rx) = crossbeam_channel::bounded::<Pose>(512);
    let discard_rx = pose_rx.clone();
    thread::spawn(move || {
        let link = link;
        let mut scratch = [0u8; 1024];
        let mut parser = FrameParser::default();
        let mut payload = vec![0u8; FRAME_MAX_PAYLOAD];
        loop {
            let n = match link.read_bulk(&mut scratch) {
                Ok(n) => n,
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::TimedOut | io::ErrorKind::Interrupted
                    ) =>
                {
                    continue;
                }
                Err(e) => {
                    eprintln!("pose stream stopped: {e}");
                    break;
                }
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
    commands.insert_resource(PuckLink { pose_rx });
    // JPEG sender thread: owns the shared bulk handle's OUT side, encodes off
    // the render thread, fragments, and paces each fragment across the frame
    // interval. Returned buffers recycle through free_tx.
    let (frame_tx, frame_rx) = crossbeam_channel::bounded::<Frame>(2);
    let (free_tx, free_rx) = crossbeam_channel::unbounded::<Box<[u8]>>();
    let return_tx = free_tx.clone();
    let skipped = Arc::new(AtomicU32::new(0));
    let skipped_tx = skipped.clone();
    thread::spawn(move || {
        let mut writer = writer;
        let mut out: Vec<u8> = Vec::with_capacity(VIDEO_JPEG_MAX);
        let mut crop: Vec<u8> = vec![0u8; VIDEO_PANEL_W * VIDEO_H * 4]; // one eye at most
        let mut lens = [0usize; VIDEO_RECTS_MAX];
        let mut cached: Vec<u8> = Vec::new(); // last frame sent, for the heartbeat
        let mut cached_rects = [Rect {
            eye: 0,
            x: 0,
            y: 0,
            w: 0,
            h: 0,
        }; VIDEO_RECTS_MAX];
        let mut cached_lens = [0usize; VIDEO_RECTS_MAX];
        let mut cached_n = 0usize;
        let mut cached_id = 0u16;
        let mut last_sent = Instant::now();
        let interval = Duration::from_micros(1_000_000 / VIDEO_TARGET_FPS);
        // Frames are paced on a fixed grid, one interval apart, so a frame that
        // is one small fragment neither waits out its own interval before the
        // first write nor lets the rate drift above the target.
        let mut grid = Instant::now();
        let mut log_at = Instant::now();
        let (mut sent, mut big, mut enc_us, mut enc_n, mut rects_sent) =
            (0u32, 0u32, 0u64, 0u32, 0u32);
        let mut txerr = 0u64;
        let mut fatal_error = false;
        loop {
            match frame_rx.recv_timeout(Duration::from_millis(50)) {
                Ok(frm) => {
                    let frame_start = grid.max(Instant::now());
                    grid = frame_start + interval;
                    out.clear();
                    // One JPEG per rectangle, concatenated in `out`: the device
                    // slices the blob back apart by the lengths in the table.
                    let mut ok = true;
                    let mut n = 0usize;
                    for r in frm.rects.iter().take(frm.rects_n) {
                        let len = r.w as usize * r.h as usize * 4;
                        crop_rgba(&mut crop[..len], &frm.rgba, r);
                        let before = out.len();
                        let mut enc = jpeg_encoder::Encoder::new(&mut out, JPEG_QUALITY);
                        enc.set_sampling_factor(jpeg_encoder::SamplingFactor::F_2_2); // 4:2:0
                        match enc.encode(&crop[..len], r.w, r.h, jpeg_encoder::ColorType::Rgba) {
                            Ok(()) => {
                                lens[n] = out.len() - before;
                                n += 1;
                            }
                            Err(_) => {
                                ok = false;
                                break;
                            }
                        }
                    }
                    enc_us += frame_start.elapsed().as_micros() as u64;
                    enc_n += 1;
                    let _ = return_tx.send(frm.rgba);
                    // `big`: frames skipped rather than truncated — an encode
                    // error, a rectangle that did not encode, or more bytes than
                    // the device will assemble.
                    if !ok || n != frm.rects_n || out.len() > VIDEO_JPEG_MAX {
                        big += 1;
                    } else {
                        let result = send_rects(&mut writer, frm.id, &frm.rects[..n], &lens[..n])
                            .and_then(|_| {
                                send_blob(&mut writer, frm.id, &out, frame_start, interval)
                            });
                        match result {
                            Ok(()) => {
                                cached.clear();
                                cached.extend_from_slice(&out);
                                cached_rects = frm.rects;
                                cached_lens = lens;
                                cached_n = n;
                                cached_id = frm.id;
                                rects_sent += n as u32;
                                sent += 1;
                                last_sent = Instant::now();
                            }
                            Err(e) if e.kind() == io::ErrorKind::TimedOut => {}
                            Err(e) => {
                                txerr += 1;
                                eprintln!("video stream stopped: {e}");
                                fatal_error = true;
                            }
                        }
                    }
                }
                Err(crossbeam_channel::RecvTimeoutError::Timeout) => {
                    // Static scene: the render thread enqueues nothing, so keep
                    // the device's idle RGB cycle away with a slow re-send of
                    // the last frame — rectangles included, so the device paints
                    // what it painted before rather than nothing.
                    if cached_n > 0
                        && last_sent.elapsed() >= Duration::from_millis(VIDEO_HEARTBEAT_MS)
                    {
                        let now = Instant::now();
                        grid = now + interval;
                        let result = send_rects(
                            &mut writer,
                            cached_id,
                            &cached_rects[..cached_n],
                            &cached_lens[..cached_n],
                        )
                        .and_then(|_| send_blob(&mut writer, cached_id, &cached, now, interval));
                        match result {
                            Ok(()) => last_sent = now,
                            Err(e) if e.kind() == io::ErrorKind::TimedOut => {}
                            Err(e) => {
                                txerr += 1;
                                eprintln!("video stream stopped: {e}");
                                fatal_error = true;
                            }
                        }
                    }
                }
                Err(crossbeam_channel::RecvTimeoutError::Disconnected) => break,
            }
            if log_at.elapsed() >= Duration::from_secs(1) || fatal_error {
                let enc_ms = if enc_n == 0 {
                    0.0
                } else {
                    enc_us as f64 / enc_n as f64 / 1000.0
                };
                println!(
                    "video {sent} fps skip {} big {big} enc {enc_ms:.1} ms rects {rects_sent} txerr {txerr}",
                    skipped_tx.load(Ordering::Relaxed),
                );
                sent = 0;
                big = 0;
                rects_sent = 0;
                enc_us = 0;
                enc_n = 0;
                log_at = Instant::now();
                if fatal_error {
                    break;
                }
            }
        }
    });

    // One persistent readback entity, with its `Readback` component attached and
    // detached by throttle_readback() as the sender allows. Spawn once and never
    // despawn: no entity churn, and no wasted GPU copies.
    let readback_entity = commands.spawn_empty().id();
    commands.insert_resource(VideoTx {
        frame_tx,
        free_tx,
        free_rx,
        skipped,
        prev: vec![0u8; VIDEO_W * 4 * VIDEO_H].into_boxed_slice(),
        prev_valid: false,
        full_at: Instant::now(),
        keyframe_step: 0,
        frame_id: 0,
        readback_entity,
        target,
    });
}

// Readback fires when the requested texture copy completes. Copy it into a
// pooled buffer and hand it to the sender thread; an unchanged frame enqueues
// nothing at all (the sender's heartbeat covers the static scene).
fn on_video_readback(readback: On<ReadbackComplete>, mut video: ResMut<VideoTx>) {
    let data = &readback.data;
    // Tight-pack invariant: 640*4 = 2560 is a multiple of the 256-byte row
    // alignment, so the readback is exactly 614400 bytes with no stride
    // padding. Fails loudly in debug/test if the GPU layout ever changes;
    // the release early-return below keeps the live path from slicing OOB.
    debug_assert_eq!(data.len(), VIDEO_W * 4 * VIDEO_H, "readback stride changed");
    if data.len() < VIDEO_W * 4 * VIDEO_H {
        return;
    }
    // The sender is the pacer. A readback it cannot take right away is not
    // consumed at all: leaving `prev` alone means the next one is compared
    // against the frame the device painted, so the difference widens into that
    // next frame's rectangles instead of being lost.
    if video.frame_tx.is_full() {
        video.skipped.fetch_add(1, Ordering::Relaxed);
        return;
    }
    let src = &data[..VIDEO_W * 4 * VIDEO_H];
    // A repair pass walks the canvas a quadrant at a time; a frame that is not
    // part of one carries only what changed.
    let starting = video.full_at.elapsed() >= Duration::from_millis(VIDEO_KEYFRAME_MS);
    if starting {
        video.keyframe_step = 0;
        video.full_at = Instant::now();
    }
    let repairing = starting || video.keyframe_step > 0;
    if video.prev_valid && !repairing && &video.prev[..] == src {
        return; // static scene: no buffer taken, nothing enqueued
    }
    // Nothing on the device is known to match the first frame (no previous
    // readback), so that one goes as the whole canvas at once.
    let (mut rects, mut rects_n) = if video.prev_valid {
        dirty_rects(src, &video.prev, false)
    } else {
        dirty_rects(src, src, true)
    };
    let repair_step = video.keyframe_step;
    if repairing && video.prev_valid {
        // This pass's quadrant, per eye — unless the dirty box already covers
        // it, in which case the frame is painting that area anyway.
        for eye in 0..2u8 {
            let q = repair_slice(eye, repair_step);
            let covered = rects[..rects_n].iter().any(|r| {
                r.eye == eye
                    && r.x <= q.x
                    && r.y <= q.y
                    && r.x + r.w >= q.x + q.w
                    && r.y + r.h >= q.y + q.h
            });
            if !covered && rects_n < VIDEO_RECTS_MAX {
                rects[rects_n] = q;
                rects_n += 1;
            }
        }
    }
    let mut buf = video
        .free_rx
        .try_recv()
        .unwrap_or_else(|_| vec![0u8; VIDEO_W * 4 * VIDEO_H].into_boxed_slice());
    buf.copy_from_slice(src);
    if rects_n == 0 {
        let _ = video.free_tx.send(buf); // nothing changed, and no keyframe due
        return;
    }
    let frame = video.frame_id;
    match video.frame_tx.try_send(Frame {
        rgba: buf,
        rects,
        rects_n,
        id: frame,
    }) {
        // Only a frame the sender took becomes the baseline: `prev` is defined
        // as "what the panels will show".
        Ok(()) => {
            video.prev.copy_from_slice(src);
            video.prev_valid = true;
            video.frame_id = frame.wrapping_add(1);
            if repairing && video.prev_valid {
                // Only a slice that went out advances the pass: a frame the
                // sender could not take is retried on the next readback instead
                // of leaving a hole in the repair.
                video.keyframe_step = (repair_step + 1) % VIDEO_KEYFRAME_STEPS;
            }
        }
        // Lost the race with the sender (it filled up in between): same deal as
        // an early skip — leave the baseline where it was.
        Err(crossbeam_channel::TrySendError::Full(frm)) => {
            video.skipped.fetch_add(1, Ordering::Relaxed);
            let _ = video.free_tx.send(frm.rgba);
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

    // The recenter key has no wire bytes to pin, so run the real system and read
    // the rig: without R the rig is the pose; on R it becomes the reference view
    // and stays in that frame afterwards. The last step is what pins the bias to
    // the LEFT of the pose — a right-multiplied bias is identical at the press
    // and different forever after.
    #[test]
    fn r_recenters_the_rig_and_poses_afterwards_are_relative_to_it() {
        let pose = |seq: u32, q: Quat| Pose {
            seq,
            stamp: 0,
            q,
            gyro: Vec3::ZERO,
        };
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.init_resource::<ButtonInput<KeyCode>>();
        app.init_resource::<ViewBias>();
        let (tx, rx) = crossbeam_channel::bounded::<Pose>(4);
        app.insert_resource(PuckLink { pose_rx: rx });
        let rig = app.world_mut().spawn((LookRig, Transform::IDENTITY)).id();
        app.add_systems(Update, update);
        // The smoothing slerp needs real time to converge; 50 ms a frame makes
        // it settle in two, so the assertions below are about the bias, not tau.
        let step = |app: &mut App, pose: Pose| {
            tx.try_send(pose).unwrap();
            thread::sleep(Duration::from_millis(50));
            app.update();
        };
        let rotation = |app: &App| app.world().entity(rig).get::<Transform>().unwrap().rotation;

        let turned = Quat::from_rotation_y(0.7);
        step(&mut app, pose(1, turned));
        assert!(
            rotation(&app).angle_between(turned) < 2e-3,
            "{:?}",
            rotation(&app)
        );

        // R: the orientation the puck is in now becomes the reference view.
        let pressed = Quat::from_rotation_x(-0.4) * turned;
        let keys = &mut app.world_mut().resource_mut::<ButtonInput<KeyCode>>();
        keys.press(KeyCode::KeyR);
        step(&mut app, pose(2, pressed));
        assert!(
            rotation(&app).angle_between(Quat::IDENTITY) < 2e-3,
            "{:?}",
            rotation(&app)
        );

        // Released: a later pose shows up as what it differs from the captured
        // one by, composed on the left (world frame), and settles there.
        let keys = &mut app.world_mut().resource_mut::<ButtonInput<KeyCode>>();
        keys.release(KeyCode::KeyR);
        keys.clear();
        let later = Quat::from_rotation_z(0.25) * pressed;
        for seq in 3..6 {
            step(&mut app, pose(seq, later));
        }
        let expected = pressed.conjugate() * later;
        assert!(
            rotation(&app).angle_between(expected) < 2e-3,
            "{:?} != {expected:?}",
            rotation(&app)
        );
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

    fn poke(buf: &mut [u8], eye: usize, tx: usize, ty: usize, dx: usize, dy: usize) {
        let x = eye * VIDEO_PANEL_W + tx * VIDEO_TILE + dx;
        let y = ty * VIDEO_TILE + dy;
        buf[y * VIDEO_W * 4 + x * 4] = 0xFF;
    }

    // Wire layout of the rectangle table for one frame, pinned to the same
    // literal bytes the device-side test (tests/video_test.cpp) asserts: magic,
    // FRAME_VIDEO_RECT, len, frame id, rect count, then the rects.
    #[test]
    fn video_rect_frame_wire_layout_is_pinned() {
        let rects = [
            Rect {
                eye: 0,
                x: 16,
                y: 32,
                w: 48,
                h: 64,
            },
            Rect {
                eye: 1,
                x: 304,
                y: 0,
                w: 16,
                h: 16,
            },
        ];
        let lens = [111usize, 7usize];
        let mut wire = Vec::new();
        send_rects(&mut wire, 0x1234, &rects, &lens).unwrap();
        let mut want = vec![
            0x55, 0xAA, 0x05, 0x1A, 0x00, // magic, FRAME_VIDEO_RECT, len = 26
            0x34, 0x12, 0x02, 0x00, // frame, rects = 2
            0x00, 0x10, 0x00, 0x20, 0x00, 0x30, 0x00, 0x40, 0x00, 0x6F, 0x00, 0x01, 0x30, 0x01,
            0x00, 0x00, 0x10, 0x00, 0x10, 0x00, 0x07, 0x00,
        ];
        let mut crc: u16 = 0xFFFF;
        for &b in &wire[2..5 + 4 + 2 * VIDEO_RECT_LEN] {
            crc = frame_crc_feed(crc, b);
        }
        want.extend_from_slice(&crc.to_le_bytes());
        assert_eq!(wire, want);
    }

    // The rectangles must cover exactly the tiles that changed, per eye: one
    // that is too small leaves stale pixels on the panel, one that is too large
    // pays decode and blit for pixels that did not move.
    #[test]
    fn dirty_rects_cover_exactly_the_changed_tiles() {
        let base = vec![0u8; VIDEO_W * 4 * VIDEO_H];
        // Nothing changed: no rectangles, which the caller turns into "send
        // nothing at all".
        assert_eq!(dirty_rects(&base, &base, false).1, 0);

        // One tile in each eye: two rectangles, each exactly that tile.
        let mut cur = base.clone();
        poke(&mut cur, 0, 3, 2, 0, 0);
        poke(&mut cur, 1, 19, 14, VIDEO_TILE - 1, VIDEO_TILE - 1);
        let (rects, n) = dirty_rects(&cur, &base, false);
        assert_eq!(n, 2);
        assert_eq!(
            rects[0],
            Rect {
                eye: 0,
                x: 48,
                y: 32,
                w: 16,
                h: 16
            }
        );
        assert_eq!(
            rects[1],
            Rect {
                eye: 1,
                x: 304,
                y: 224,
                w: 16,
                h: 16
            }
        );

        // Scattered tiles in one eye collapse to their bounding box, and the
        // untouched eye contributes nothing.
        let mut cur = base.clone();
        poke(&mut cur, 1, 4, 1, 5, 5);
        poke(&mut cur, 1, 9, 4, 1, 1);
        let (rects, n) = dirty_rects(&cur, &base, false);
        assert_eq!(n, 1);
        assert_eq!(
            rects[0],
            Rect {
                eye: 1,
                x: 64,
                y: 16,
                w: 96,
                h: 64
            }
        );

        // The startup/anonymous case is both whole panels whatever changed.
        let (rects, n) = dirty_rects(&base, &base, true);
        assert_eq!(n, 2);
        for (i, r) in rects.iter().take(n).enumerate() {
            assert_eq!(
                *r,
                Rect {
                    eye: i as u8,
                    x: 0,
                    y: 0,
                    w: 320,
                    h: 240
                }
            );
        }
    }

    // The repair pass is the whole canvas, a quadrant per frame per eye: four
    // steps must tile the panel exactly, stay on the MCU grid, and add up to
    // nothing more than the canvas it replaces.
    #[test]
    fn repair_slices_tile_the_panel() {
        let mut area = 0;
        for step in 0..VIDEO_KEYFRAME_STEPS {
            for eye in 0..2u8 {
                let q = repair_slice(eye, step);
                assert_eq!(q.eye, eye);
                assert_eq!((q.x % 16, q.y % 16, q.w % 16, q.h % 16), (0, 0, 0, 0));
                assert!(
                    q.x + q.w <= 320 && q.y + q.h <= 240,
                    "{q:?} outside the panel"
                );
                area += q.w as usize * q.h as usize;
                // No overlap with the other quadrants of the same eye.
                for other in 0..VIDEO_KEYFRAME_STEPS {
                    if other == step {
                        continue;
                    }
                    let o = repair_slice(eye, other);
                    let disjoint = q.x + q.w <= o.x
                        || o.x + o.w <= q.x
                        || q.y + q.h <= o.y
                        || o.y + o.h <= q.y;
                    assert!(disjoint, "{q:?} overlaps {o:?}");
                }
            }
        }
        assert_eq!(area, 2 * 320 * 240, "the slices of one eye are the canvas");
    }

    // The crop is what the encoder is handed, in canvas coordinates: an eye's
    // rectangle is offset by that eye's half of the canvas.
    #[test]
    fn crop_rgba_packs_the_rectangle_rows_tight() {
        let mut rgba = vec![0u8; VIDEO_W * 4 * VIDEO_H];
        for y in 0..VIDEO_H {
            for x in 0..VIDEO_W {
                let o = (y * VIDEO_W + x) * 4;
                rgba[o] = (x % 251) as u8;
                rgba[o + 1] = (y % 251) as u8;
                rgba[o + 2] = 0x5A;
            }
        }
        let r = Rect {
            eye: 1,
            x: 32,
            y: 48,
            w: 32,
            h: 16,
        };
        let mut out = vec![0u8; r.w as usize * r.h as usize * 4];
        crop_rgba(&mut out, &rgba, &r);
        for row in 0..16usize {
            for col in 0..32usize {
                let o = (row * 32 + col) * 4;
                assert_eq!(out[o], ((VIDEO_PANEL_W + 32 + col) % 251) as u8);
                assert_eq!(out[o + 1], ((48 + row) % 251) as u8);
                assert_eq!(out[o + 2], 0x5A);
            }
        }
    }

    // Fragments come out in order with the expected lengths, and the parser puts
    // them back together byte for byte (the device's assembler mirrors this).
    #[test]
    fn send_blob_fragments_and_the_parser_reassembles_them() {
        let blob: Vec<u8> = (0..VIDEO_FRAG_MAX * 2 + 17)
            .map(|i| (i * 7) as u8)
            .collect();
        let mut wire = Vec::new();
        // Duration::ZERO: nothing to pace against in a test.
        send_blob(&mut wire, 9, &blob, Instant::now(), Duration::ZERO).unwrap();
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
