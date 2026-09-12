use std::net::UdpSocket;

use bevy::prelude::*;

#[derive(Resource)]
struct Socket(UdpSocket);

#[derive(Component)]
struct Puck;

// must match device/config.h UDP_PORT
const UDP_PORT: u16 = 3333;
const BIND_HOST: &str = "0.0.0.0";
// must match QUAT_PACKET_SIZE (seq + micros + xyzw)
const PACKET_SIZE: usize = 24;

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

fn parse_packet(raw: [u8; PACKET_SIZE]) -> (u32, u32, Quat) {
    let seq = u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]);
    let stamp = u32::from_le_bytes([raw[4], raw[5], raw[6], raw[7]]);
    let f = |i: usize| f32::from_le_bytes([raw[i], raw[i + 1], raw[i + 2], raw[i + 3]]);
    (seq, stamp, Quat::from_xyzw(f(8), f(12), f(16), f(20)))
}

fn update(
    query: Single<&mut Transform, With<Puck>>,
    socket: Res<Socket>,
    mut last_seq: Local<Option<u32>>,
) {
    // Drain to latest: 24-byte LE seq+stamp+xyzw packet, ESP32 and x86 both LE
    let mut buf = [0u8; PACKET_SIZE];
    let mut latest: Option<(u32, u32, Quat)> = None;

    while let Ok(PACKET_SIZE) = socket.0.recv(&mut buf) {
        latest = Some(parse_packet(buf));
    }

    if let Some((seq, _stamp, q)) = latest {
        if let Some(prev) = *last_seq {
            if seq != prev.wrapping_add(1) {
                println!(
                    "seq gap: prev {prev} got {seq} (lost {})",
                    seq.wrapping_sub(prev.wrapping_add(1))
                );
            }
        }
        *last_seq = Some(seq);
        query.into_inner().rotation = q;
    }
}

fn setup_world(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Puck,
        Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.8))),
        Transform::from_xyz(0.0, 0.5, 0.0),
    ));

    commands.spawn((PointLight::default(), Transform::from_xyz(4.0, 8.0, 4.0)));

    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
