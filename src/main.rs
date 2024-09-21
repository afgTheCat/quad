mod control;
mod rigid_body;
mod ui;

use bevy::{
    input::{
        gamepad::{GamepadAxisChangedEvent, GamepadEvent},
        keyboard::KeyboardInput,
    },
    prelude::*,
};
use control::handle_keyboard_events;

#[derive(Debug, Resource)]
struct ControllerPosition(Vec4);

impl ControllerPosition {
    fn set_throttle(&mut self, value: f32) {
        self.0[0] = value;
    }

    fn set_yaw(&mut self, value: f32) {
        self.0[1] = value;
    }

    fn set_pitch(&mut self, value: f32) {
        self.0[2] = value;
    }

    fn set_roll(&mut self, value: f32) {
        self.0[3] = value;
    }
}

fn handle_controller_axis(
    controller: &mut ResMut<ControllerPosition>,
    axis: &GamepadAxisChangedEvent,
) {
    let value = axis.value;
    match axis.axis_type {
        GamepadAxisType::RightStickY => {
            controller.set_throttle(value);
        }
        GamepadAxisType::RightStickX => {
            controller.set_yaw(value);
        }
        GamepadAxisType::LeftStickY => {
            controller.set_pitch(value);
        }
        GamepadAxisType::LeftStickX => {
            controller.set_roll(value);
        }
        _ => {}
    }
    println!("{:?}", controller);
}

/// set up a simple 3D scene
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // circular base
    commands.spawn(PbrBundle {
        mesh: meshes.add(Circle::new(4.0)),
        material: materials.add(Color::WHITE),
        transform: Transform::from_rotation(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)),
        ..default()
    });
    // cube
    commands.spawn(PbrBundle {
        mesh: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
        material: materials.add(Color::srgb_u8(124, 144, 255)),
        transform: Transform::from_xyz(0.0, 0.5, 0.0),
        ..default()
    });
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
        .insert_resource(ControllerPosition(Vec4::default()))
        .add_systems(Startup, setup)
        .add_systems(Update, handle_keyboard_events)
        .run();
}
