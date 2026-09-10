use std::net::UdpSocket;

use bevy::prelude::*;

#[derive(Resource)]
struct Socket(Option<UdpSocket>);

const BIND_ADDR: &str = "0.0.0.0:3333";
const PACKET_SIZE: usize = 16;

pub struct CubePlugin;

impl Plugin for CubePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Socket(None));
        app.add_systems(Startup, (setup_world, setup_socket));
        app.add_systems(Update, update);
    }
}

fn setup_socket(mut socket: ResMut<Socket>) {
    match UdpSocket::bind(BIND_ADDR) {
        Ok(res_socket) => {
            res_socket.set_nonblocking(true).expect("udp nonblocking");
            socket.0 = Some(res_socket);
        }
        Err(err) => {
            socket.0 = None;
            println!("Failed to connect: {:?}", err);
        }
    }
}

fn parse_quat(raw: [u8; PACKET_SIZE]) -> Quat {
    let x = f32::from_le_bytes(raw[0..4].try_into().unwrap());
    let y = f32::from_le_bytes(raw[4..8].try_into().unwrap());
    let z = f32::from_le_bytes(raw[8..12].try_into().unwrap());
    let w = f32::from_le_bytes(raw[12..16].try_into().unwrap());
    Quat::from_xyzw(x, y, z, w)
}

fn update(mut query: Query<&mut Transform, With<Mesh3d>>, socket: Res<Socket>) {
    let Some(udp) = &socket.0 else { return };
    // Drain to latest: 16-byte LE xyzw packet, ESP32 and x86 are both LE
    let mut buf = [0u8; PACKET_SIZE];
    let mut latest: Option<Quat> = None;
    loop {
        match udp.recv(&mut buf) {
            Ok(PACKET_SIZE) => {
                latest = Some(parse_quat(buf));
            }
            _ => break,
        }
    }
    if let Some(q) = latest {
        for mut trans in &mut query {
            trans.rotation = q;
        }
    }
}

fn setup_world(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
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
