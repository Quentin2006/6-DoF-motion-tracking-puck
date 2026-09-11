use std::net::UdpSocket;

use bevy::prelude::*;

#[derive(Resource)]
struct Socket(UdpSocket);

#[derive(Component)]
struct Puck;

// must match device/config.h UDP_PORT
const UDP_PORT: u16 = 3333;
const BIND_HOST: &str = "0.0.0.0";
// must match QUAT_PACKET_SIZE
const PACKET_SIZE: usize = 16;

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

fn parse_quat(raw: [u8; PACKET_SIZE]) -> Quat {
    let f = |i: usize| f32::from_le_bytes([raw[i], raw[i + 1], raw[i + 2], raw[i + 3]]);
    Quat::from_xyzw(f(0), f(4), f(8), f(12))
}

fn update(query: Single<&mut Transform, With<Puck>>, socket: Res<Socket>) {
    // Drain to latest: 16-byte LE xyzw packet, ESP32 and x86 are both LE
    let mut buf = [0u8; PACKET_SIZE];
    let mut latest: Option<Quat> = None;

    while let Ok(PACKET_SIZE) = socket.0.recv(&mut buf) {
        latest = Some(parse_quat(buf));
    }

    if let Some(q) = latest {
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
