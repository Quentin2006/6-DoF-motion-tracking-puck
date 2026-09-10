mod net;
mod tracking;
mod world;

use bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(tracking::CubePlugin)
        .run();
}
