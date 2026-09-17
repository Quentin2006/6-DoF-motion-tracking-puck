use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use bevy::camera::RenderTarget;
use bevy::prelude::*;
use bevy::render::gpu_readback::{Readback, ReadbackComplete};
use bevy::render::render_resource::{TextureFormat, TextureUsages};

#[derive(Resource)]
struct Socket(UdpSocket);

// Marker for the camera rig driven by the tracking puck (look-around).
#[derive(Component)]
struct LookRig;

// must match device/config.h UDP_PORT
const UDP_PORT: u16 = 3333;
const BIND_HOST: &str = "0.0.0.0";
// must match device/packet.h QUAT_PACKET_LEN_V2
const PACKET_LEN_V2: usize = 36;

// Transport+render delay extrapolated from packet gyro, and display smoothing.
const PREDICT_TAU_S: f32 = 0.008;
const SMOOTH_TAU_S: f32 = 0.006;

// Video stream to the puck panel: 320x240 RGB565 over UDP 3334.
// Must match device/video.h VIDEO_* constants.
const VIDEO_UDP_PORT: u16 = 3334;
const VIDEO_W: usize = 320;
const VIDEO_H: usize = 240;
const VIDEO_PAYLOAD_MAX: usize = 1280; // 2 rows x 320 px x 2 B
const VIDEO_CHUNK_ROWS: usize = VIDEO_PAYLOAD_MAX / (VIDEO_W * 2); // 2
const VIDEO_CHUNKS: usize = VIDEO_H / VIDEO_CHUNK_ROWS; // 120
const VIDEO_MIN_INTERVAL: Duration = Duration::from_millis(200); // 5 fps cap

// UDP sender for video datagrams; peer learned from the quat stream source addr.
#[derive(Resource)]
struct VideoTx {
    socket: UdpSocket,
    peer: Option<SocketAddr>,
    last_send: Instant,
    frame_id: u16,
}

pub struct TrackPlugin;

impl Plugin for TrackPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, (setup_scene, setup_socket));
        app.add_systems(Update, update);
        app.add_observer(on_video_readback);
    }
}

fn setup_socket(mut commands: Commands) {
    let addr = format!("{BIND_HOST}:{UDP_PORT}");
    let udp = UdpSocket::bind(&addr).unwrap_or_else(|e| panic!("bind {addr}: {e:?}"));
    udp.set_nonblocking(true).expect("udp nonblocking");
    commands.insert_resource(Socket(udp));
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

fn parse_v2(raw: &[u8]) -> (u32, u32, Quat, Vec3) {
    let (seq, stamp) = header(raw);
    let q = Quat::from_xyzw(f(raw, 8), f(raw, 12), f(raw, 16), f(raw, 20));
    // Body-frame gyro rad/s.
    let gyro = Vec3::new(f(raw, 24), f(raw, 28), f(raw, 32));
    (seq, stamp, q, gyro)
}

fn update(
    camera: Single<&mut Transform, With<LookRig>>,
    socket: Res<Socket>,
    mut video: Option<ResMut<VideoTx>>,
    mut last_seq: Local<Option<u32>>,
    mut last_applied: Local<Option<Quat>>,
    time: Res<Time>,
) {
    // Drain to latest: ESP32 and x86 are both LE, and only the newest packet is
    // displayed, so nothing but the last datagram needs decoding.
    let mut buf = [0u8; PACKET_LEN_V2];
    let mut latest: Option<(u32, Quat, Vec3)> = None;
    while let Ok((n, src)) = socket.0.recv_from(&mut buf) {
        // Learn the puck's address for the video stream (same host, video port).
        if let Some(v) = video.as_mut() {
            let mut peer = src;
            peer.set_port(VIDEO_UDP_PORT);
            v.peer = Some(peer);
        }
        if n == PACKET_LEN_V2 {
            let (seq, _, q, gyro) = parse_v2(&buf);
            latest = Some((seq, q, gyro));
        }
    }
    let Some((seq, q, gyro)) = latest else {
        return;
    };

    if let Some(prev) = *last_seq {
        // Drain-to-latest normally skips ~4 packets/frame at 250Hz/60fps — not
        // loss; threshold 25 (~100ms) flags real bursts only.
        let lost = seq.wrapping_sub(prev.wrapping_add(1));
        if lost > 25 {
            println!("seq gap: prev {prev} got {seq} (lost {lost})");
        }
    }
    *last_seq = Some(seq);

    // Predict forward by PREDICT_TAU_S in the body frame (right-multiply).
    let ang = gyro.length() * PREDICT_TAU_S;
    let unnorm = if ang > 1e-6 {
        q * Quat::from_axis_angle(gyro.normalize(), ang)
    } else {
        q
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
    commands.spawn(Readback::texture(target));

    let udp = UdpSocket::bind("0.0.0.0:0").unwrap_or_else(|e| panic!("video bind: {e:?}"));
    udp.set_nonblocking(true).expect("video nonblocking");
    commands.insert_resource(VideoTx {
        socket: udp,
        peer: None,
        last_send: Instant::now() - VIDEO_MIN_INTERVAL,
        frame_id: 0,
    });
}

// Readback fires every frame; pack RGB565 and fragment into VIDEO_CHUNKS
// datagrams, throttled to 5 fps. Drop-if-behind: skip frames that arrive sooner
// than VIDEO_MIN_INTERVAL after the last send.
fn on_video_readback(readback: On<ReadbackComplete>, mut video: ResMut<VideoTx>) {
    let data = &readback.data;
    // RGBA8, 4 B per pixel; stride == row length (VIDEO_W * 4 is 256-aligned).
    if data.len() < VIDEO_W * 4 * VIDEO_H {
        return;
    }
    let now = Instant::now();
    if now.duration_since(video.last_send) < VIDEO_MIN_INTERVAL {
        return;
    }
    let Some(peer) = video.peer else {
        return;
    };
    video.last_send = now;
    let frame = video.frame_id;
    video.frame_id = frame.wrapping_add(1);

    let mut pkt = [0u8; 8 + VIDEO_PAYLOAD_MAX];
    pkt[0..2].copy_from_slice(&frame.to_le_bytes());
    pkt[4..6].copy_from_slice(&(VIDEO_CHUNKS as u16).to_le_bytes());
    pkt[6..8].copy_from_slice(&(VIDEO_PAYLOAD_MAX as u16).to_le_bytes());
    for chunk in 0..VIDEO_CHUNKS {
        pkt[2..4].copy_from_slice(&(chunk as u16).to_le_bytes());
        // One chunk = 2 rows = 1280 RGB565 bytes from 2560 RGBA8 bytes.
        let src = &data[chunk * VIDEO_PAYLOAD_MAX * 2..(chunk + 1) * VIDEO_PAYLOAD_MAX * 2];
        for (dst, px) in pkt[8..].chunks_exact_mut(2).zip(src.chunks_exact(4)) {
            let v: u16 = ((px[0] as u16 >> 3) << 11) | ((px[1] as u16 >> 2) << 5) | (px[2] as u16 >> 3);
            dst.copy_from_slice(&v.to_le_bytes());
        }
        let _ = video.socket.send_to(&pkt, peer);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Shared byte-level fixture: the committed tests/packet_fixture.txt is the
    // single source of truth for the v2 wire layout, also asserted by the C++
    // producer test (tests/packet_test.cpp) against the same bytes.
    const FIXTURE: &str = include_str!("../../tests/packet_fixture.txt");

    #[test]
    fn parse_v2_roundtrips_the_committed_fixture() {
        let hex: String = FIXTURE
            .lines()
            .find(|l| !l.trim().is_empty() && !l.trim_start().starts_with('#'))
            .unwrap()
            .chars()
            .filter(|c| c.is_ascii_hexdigit())
            .collect();
        let bytes: Vec<u8> = hex
            .as_bytes()
            .chunks(2)
            .map(|c| u8::from_str_radix(std::str::from_utf8(c).unwrap(), 16).unwrap())
            .collect();
        assert_eq!(bytes.len(), PACKET_LEN_V2);
        let (seq, stamp, q, gyro) = parse_v2(&bytes);
        assert_eq!((seq, stamp), (42, 1000));
        assert_eq!(q, Quat::from_xyzw(0.0, 0.0, 0.0, 1.0));
        let e = gyro - Vec3::new(0.5, -0.25, 1.5);
        assert!(e.length() < 1e-6, "gyro mismatch: {gyro:?}");
    }
}
