use bevy::prelude::*;

use super::net::Socket;

// A core feature of bevy is plugins, it allows for modularity
pub struct CubePlugin;

// All a plugin is doing is adding stuff to your app
impl Plugin for CubePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Socket(None));
        app.add_systems(Startup, (super::world::setup_world, super::net::setup_socket));
        app.add_systems(Update, update);
    }
}

// Systems can also be mutable, allowing for modification of data
fn update(mut query: Query<&mut Transform, With<Mesh3d>>, socket: Res<Socket>) {
    let Some(udp) = &socket.0 else { return };
    // Drain to latest: 16-byte LE xyzw packet, ESP32 and x86 are both LE
    let mut buf = [0u8; super::net::PACKET_SIZE];
    let mut latest: Option<Quat> = None;
    loop {
        match udp.recv(&mut buf) {
            Ok(super::net::PACKET_SIZE) => {
                latest = Some(super::net::parse_quat(buf));
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
