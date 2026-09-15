use std::net::UdpSocket;

use bevy::prelude::*;

#[derive(Resource)]
struct Socket(UdpSocket);

#[derive(Component)]
struct Puck;

// must match device/config.h UDP_PORT
const UDP_PORT: u16 = 3333;
const BIND_HOST: &str = "0.0.0.0";
// must match device/config.h QUAT_PACKET_SIZE / QUAT_PACKET_SIZE_V2
// (seq + micros + xyzw = 24 legacy; + gyro xyz = 36 v2)
const PACKET_LEN_V1: usize = 24;
const PACKET_LEN_V2: usize = 36;

// Transport+render delay extrapolated from packet gyro, and display smoothing.
const PREDICT_TAU_S: f32 = 0.008;
const SMOOTH_TAU_S: f32 = 0.006;

pub struct CubePlugin;

impl Plugin for CubePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, (setup_world, setup_socket));
        app.add_systems(Update, update);
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

fn parse_v1(raw: &[u8]) -> (u32, u32, Quat) {
    let (seq, stamp) = header(raw);
    (seq, stamp, Quat::from_xyzw(f(raw, 8), f(raw, 12), f(raw, 16), f(raw, 20)))
}

fn parse_v2(raw: &[u8]) -> (u32, u32, Quat, Vec3) {
    let (seq, stamp) = header(raw);
    let q = Quat::from_xyzw(f(raw, 8), f(raw, 12), f(raw, 16), f(raw, 20));
    // Body-frame gyro rad/s.
    let gyro = Vec3::new(f(raw, 24), f(raw, 28), f(raw, 32));
    (seq, stamp, q, gyro)
}

fn update(
    query: Single<&mut Transform, With<Puck>>,
    socket: Res<Socket>,
    mut last_seq: Local<Option<u32>>,
    mut last_applied: Local<Option<Quat>>,
    time: Res<Time>,
) {
    // Drain to latest: ESP32 and x86 both LE.
    let mut buf = [0u8; PACKET_LEN_V2];
    let mut latest: Option<(u32, u32, Quat, Option<Vec3>)> = None;

    while let Ok(n) = socket.0.recv(&mut buf) {
        match n {
            PACKET_LEN_V1 => {
                let (seq, stamp, q) = parse_v1(&buf[..PACKET_LEN_V1]);
                latest = Some((seq, stamp, q, None));
            }
            PACKET_LEN_V2 => {
                let (seq, stamp, q, gyro) = parse_v2(&buf);
                latest = Some((seq, stamp, q, Some(gyro)));
            }
            _ => {}
        }
    }

    if let Some((seq, _stamp, q, gyro)) = latest {
        if let Some(prev) = *last_seq {
            // Drain-to-latest normally skips ~4 packets/frame at 250Hz/60fps — not loss; threshold 25 (~100ms) flags real bursts only.
            let lost = seq.wrapping_sub(prev.wrapping_add(1));
            if lost > 25 {
                println!("seq gap: prev {prev} got {seq} (lost {lost})");
            }
        }
        *last_seq = Some(seq);
        // Only overwrite when the packet carried gyro (v2).
        if let Some(w) = gyro {
            *last_gyro = Some(w);
        }

        // Predict forward by PREDICT_TAU_S in the body frame (right-multiply).
        // Legacy v1 packets carry no gyro, so they skip prediction.
        let q_show = match gyro {
            Some(w) => {
                let ang = w.length() * PREDICT_TAU_S;
                if ang > 1e-6 {
                    q * Quat::from_axis_angle(w.normalize(), ang)
                } else {
                    q
                }
            }
            None => q,
        };

        let dt_render = time.delta_secs();
        let alpha = (1.0 - (-dt_render / SMOOTH_TAU_S).exp()).clamp(0.0, 1.0);
        let q_out = match *last_applied {
            Some(p) => p.slerp(q_show, alpha).normalize(),
            None => q_show.normalize(),
        };

        query.into_inner().rotation = q_out;
        *last_applied = Some(q_out);
    }
}

fn setup_world(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Puck,
        Mesh3d(meshes.add(Cuboid::new(2.9 / 2., 0.1 / 2., 5.7 / 2.))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.8))),
        Transform::from_xyz(0.0, 0.5, 0.0),
    ));

    commands.spawn((PointLight::default(), Transform::from_xyz(4.0, 8.0, 4.0)));

    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
