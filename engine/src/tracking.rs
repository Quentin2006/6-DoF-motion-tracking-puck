use std::net::UdpSocket;

use bevy::prelude::*;

#[derive(Resource)]
struct Socket(UdpSocket);

// Marker for the camera rig driven by the tracking puck (look-around).
#[derive(Component)]
struct LookRig;

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

pub struct TrackPlugin;

impl Plugin for TrackPlugin {
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
    (
        seq,
        stamp,
        Quat::from_xyzw(f(raw, 8), f(raw, 12), f(raw, 16), f(raw, 20)),
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

        // Predict forward by PREDICT_TAU_S in the body frame (right-multiply).
        // Legacy v1 packets carry no gyro, so they skip prediction.
        let unnorm = match gyro {
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

        let alpha = (1.0 - (-time.delta_secs() / SMOOTH_TAU_S).exp()).clamp(0.0, 1.0);
        let q_out = match *last_applied {
            Some(p) => p.slerp(unnorm, alpha),
            None => unnorm,
        }
        .normalize();

        camera.into_inner().rotation = q_out;
        *last_applied = Some(q_out);
    }
}

fn setup_world(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Static reference object so the camera rig's motion is visible.
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(2.9 / 2., 0.1 / 2., 5.7 / 2.))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.8))),
        Transform::from_xyz(0.0, 0.5, -6.0),
    ));

    commands.spawn((PointLight::default(), Transform::from_xyz(4.0, 8.0, 4.0)));

    // The puck drives this camera rig. Quaternion maps directly to orientation;
    // translation stays at the eye position. Rest quat (identity) looks down -Z,
    // so the reference object sits in front of it.
    commands.spawn((
        LookRig,
        Camera3d::default(),
        Transform::from_xyz(0.0, 1.5, 0.0),
    ));
}

#[cfg(test)]
mod tests {
    use super::*;

    // Shared byte-level fixture: the committed tests/packet_fixture.txt is the
    // single source of truth for the v2 wire layout, also asserted by the C++
    // producer test (tests/packet_test.cpp) against the same bytes.
    const FIXTURE: &str = include_str!("../../tests/packet_fixture.txt");

    fn fixture_hex() -> &'static str {
        FIXTURE
            .lines()
            .find(|l| {
                let t = l.trim();
                !t.is_empty() && !t.starts_with('#')
            })
            .unwrap()
            .trim()
    }

    fn hex_bytes(s: &str) -> Vec<u8> {
        let clean: String = s.chars().filter(|c| c.is_ascii_hexdigit()).collect();
        clean
            .as_bytes()
            .chunks(2)
            .map(|ch| u8::from_str_radix(std::str::from_utf8(ch).unwrap(), 16).unwrap())
            .collect()
    }

    #[test]
    fn parse_v2_roundtrips_the_committed_fixture() {
        let bytes = hex_bytes(fixture_hex());
        assert_eq!(bytes.len(), PACKET_LEN_V2);
        let (seq, stamp, q, gyro) = parse_v2(&bytes);
        assert_eq!((seq, stamp), (42, 1000));
        assert_eq!(q, Quat::from_xyzw(0.0, 0.0, 0.0, 1.0));
        let e = gyro - Vec3::new(0.5, -0.25, 1.5);
        assert!(e.length() < 1e-6, "gyro mismatch: {gyro:?}");
    }

    #[test]
    fn parse_v1_roundtrips_identity_quat() {
        // v1: seq | stamp | quat xyzw (24 bytes), no gyro.
        let mut b = Vec::new();
        for v in [42u32.to_le_bytes(), 1000u32.to_le_bytes()] {
            b.extend_from_slice(&v);
        }
        for v in [0.0f32, 0.0f32, 0.0f32, 1.0f32] {
            b.extend_from_slice(&v.to_le_bytes());
        }
        let (seq, stamp, q) = parse_v1(&b);
        assert_eq!((seq, stamp), (42, 1000));
        assert_eq!(q, Quat::from_xyzw(0.0, 0.0, 0.0, 1.0));
    }

    #[test]
    fn header_decodes_seq_and_stamp_little_endian() {
        let mut b = Vec::new();
        b.extend_from_slice(&42u32.to_le_bytes());
        b.extend_from_slice(&1000u32.to_le_bytes());
        assert_eq!(header(&b), (42, 1000));
    }
}
