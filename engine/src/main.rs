mod tracking;
mod usb_bulk;

use bevy::prelude::*;

fn main() {
    if let Some(result) = usb_bulk::run_benchmark_if_requested() {
        if let Err(e) = result {
            eprintln!("bulk benchmark failed: {e}");
            std::process::exit(1);
        }
        return;
    }

    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(tracking::TrackPlugin)
        .run();
}
