use std::{f32::consts::PI, net::UdpSocket};

use bevy::prelude::*;

/*
* NOTE: Resources
*/
// These provide global state
#[derive(Resource)]
struct Socket(UdpSocket);

/*
* NOTE: Components
*/
// Components are just structs that systems can be called on
//
// #[derive(Component)]
// struct Cube;
//
// #[derive(Component)]
// struct Rotation(Quat);

/*
* NOTE: Systems
*/

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Spawn the cube mesh and material
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.8))),
        Transform::from_xyz(0.0, 0.5, 0.0),
    ));

    // Spawn a light so the cube is visible
    commands.spawn((PointLight::default(), Transform::from_xyz(4.0, 8.0, 4.0)));

    // Spawn a 3D camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

// Systems can also be mutable, allowing for modification of data
fn update(mut query: Query<&mut Transform, With<Mesh3d>>) {
    // Jsut have to make a connection with the device, then directly update the transformations rotation
    for mut trans in &mut query {
        trans.rotation *= Quat::from_rotation_x(1. * PI / 180.)
    }
}
/*
* NOTE: Plugins
*/
// A core feature of bevy is plugins, it allows for modularity
pub struct CubePlugin;

// All a plugin is doing is adding stuff to your app
impl Plugin for CubePlugin {
    fn build(&self, app: &mut App) {
        // app.insert_resource(GreetTimer(Timer::from_seconds(2.0, TimerMode::Repeating)));
        app.add_systems(Startup, setup);
        app.add_systems(Update, update);
    }
}

/*
* NOTE: Main
*/
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(CubePlugin)
        .run();
}
