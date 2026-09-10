use std::net::UdpSocket;

use bevy::prelude::{Quat, ResMut, Resource};

#[derive(Resource)]
pub struct Socket(pub Option<UdpSocket>);

pub const BIND_ADDR: &str = "0.0.0.0:3333";
pub const PACKET_SIZE: usize = 16;

pub fn setup_socket(mut socket: ResMut<Socket>) {
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

pub fn parse_quat(raw: [u8; PACKET_SIZE]) -> Quat {
    let x = f32::from_le_bytes(raw[0..4].try_into().unwrap());
    let y = f32::from_le_bytes(raw[4..8].try_into().unwrap());
    let z = f32::from_le_bytes(raw[8..12].try_into().unwrap());
    let w = f32::from_le_bytes(raw[12..16].try_into().unwrap());
    Quat::from_xyzw(x, y, z, w)
}
