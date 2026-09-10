mod tracking;

use bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(tracking::CubePlugin)
        .run();
}
