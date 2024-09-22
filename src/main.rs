mod control;
mod rigid_body;
mod ui;

use bevy::prelude::*;
use control::handle_keyboard_events;
use rigid_body::{debug_move_cube, setup_rigid_body_context};

/// set up a simple 3D scene
fn setup(mut commands: Commands) {
    // light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });
    // camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-2.5, 4.5, 9.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(Startup, setup_rigid_body_context)
        .add_systems(Update, handle_keyboard_events)
        .add_systems(Update, debug_move_cube)
        .run();
}
