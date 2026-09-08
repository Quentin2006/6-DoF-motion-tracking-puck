use bevy::prelude::*;

/*
* NOTE: Resources
*/
// These provide global state
#[derive(Resource)]
struct GreetTimer(Timer);

/*
* NOTE: Components
*/
// Components are just structs that systems can be called on
#[derive(Component)]
struct Person;
#[derive(Component)]
struct Name(String);

/*
* NOTE: Systems
*/
// Systems are function that take in Commands and does somthing with said commands
fn add_people(mut commands: Commands) {
    commands.spawn((Person, Name("Elaina Proctor".to_string())));
    commands.spawn((Person, Name("Renzo Hume".to_string())));
    commands.spawn((Person, Name("Zayna Nieves".to_string())));
}
// Systems can also be mutable, allowing for modification of data
fn update_people(mut query: Query<&mut Name, With<Person>>) {
    for mut name in &mut query {
        if name.0 == "Elaina Proctor" {
            name.0 = "Elaina Hume".to_string();
            break; // We don't need to change any other names.
        }
    }
}
// Systems can take in recorses to act upon
fn greet_people(time: Res<Time>, mut timer: ResMut<GreetTimer>, query: Query<&Name, With<Person>>) {
    // update our timer with the time elapsed since the last update
    // if that caused the timer to finish, we say hello to everyone
    if timer.0.tick(time.delta()).just_finished() {
        for name in &query {
            println!("hello {}!", name.0);
        }
    }
}

/*
* NOTE: Plugins
*/
// A core feature of bevy is plugins, it allows for modularity
pub struct HelloPlugin;

// All a plugin is doing is adding stuff to your app
impl Plugin for HelloPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(GreetTimer(Timer::from_seconds(2.0, TimerMode::Repeating)));
        // we just add poeple once at startup
        app.add_systems(Startup, add_people);
        // then we are contantly running update_people and greet_people every loop
        app.add_systems(Update, (update_people, greet_people).chain());
    }
}

/*
* NOTE: Main
*/
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(HelloPlugin)
        .run();
}
