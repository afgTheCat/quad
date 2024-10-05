use std::time::Duration;

use bevy::{
    asset::{AssetServer, Assets, Handle},
    color::Color,
    gltf::Gltf,
    math::{EulerRot, Quat, Vec3},
    pbr::{DirectionalLight, DirectionalLightBundle, StandardMaterial},
    prelude::{
        default, Camera3dBundle, Commands, Component, Gizmos, Mesh, NextState, Query, Res, ResMut,
        Resource, State, Transform,
    },
    time::Time,
};
use bevy_egui::egui::debug_text::print;
use bevy_infinite_grid::InfiniteGridBundle;
use bevy_panorbit_camera::PanOrbitCamera;
use body::Drone;
use state_packet::StatePacket;

use crate::SimState;

mod arm;
mod battery;
mod body;
mod gyro;
mod low_pass_filter;
mod motor;
mod propeller;
mod rigid_body;
mod sample_curve;
pub mod state_packet;
mod state_update_packet;

#[derive(Component)]
pub struct DroneContext {
    pub dt: Duration,
    pub time_accu: Duration, // the accumulated time between two steps + the correction from the
    pub ambient_temp: f64,
}

#[derive(Component)]
pub struct Model {}

impl Model {
    // TODO: we probably need to implement this better
    fn provide_packet(&mut self) -> StatePacket {
        todo!()
    }
}

impl DroneContext {
    fn step_context(&mut self) -> bool {
        if self.time_accu > self.dt {
            self.time_accu -= self.dt;
            true
        } else {
            false
        }
    }
}

fn sim_step(
    mut gizmos: Gizmos,
    mut query: Query<(&mut Transform, &mut DroneContext, &mut Model, &mut Drone)>,
    timer: Res<Time>,
) {
    let (mut transform, mut drone_context, mut model, mut drone) = query.single_mut();
    drone_context.time_accu += timer.delta();
    let state_packet = model.provide_packet();
    while drone_context.step_context() {
        drone.update_gyro(&state_packet, drone_context.dt.as_secs_f64());
        drone.update_physics(
            &state_packet,
            drone_context.dt.as_secs_f64(),
            drone_context.ambient_temp,
        );
    }
}

#[derive(Resource, Clone)]
pub struct DroneAssets(Handle<Gltf>);

pub fn base_setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    gltf_assets: Res<Assets<Gltf>>,
) {
    // Add a directional light to simulate the sun
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            color: Color::rgb(1.0, 1.0, 0.9), // Slightly warm sunlight
            illuminance: 100000.0, // Intensity of the light, tweak this based on your scene
            shadows_enabled: true, // Enable shadows for the sunlight
            ..Default::default()
        },
        transform: Transform {
            // Tilt the light to simulate the sun's angle (e.g., 45-degree angle)
            rotation: Quat::from_euler(EulerRot::XYZ, -std::f32::consts::FRAC_PI_4, 0.0, 0.0),
            ..Default::default()
        },
        ..Default::default()
    });

    // camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_translation(Vec3::new(0.0, 1.5, 5.0)),
            ..default()
        },
        PanOrbitCamera::default(),
    ));

    commands.spawn(InfiniteGridBundle::default());

    let drone_scene = asset_server.load("drone.glb");
    let drone_assets = DroneAssets(drone_scene);
    commands.insert_resource(drone_assets.clone());
}

pub fn setup_drone(
    drone_assets: Res<DroneAssets>,
    gltf_assets: Res<Assets<Gltf>>,
    mut next_state: ResMut<NextState<SimState>>,
) {
    // Wait until the scene is loaded
    let Some(gltf) = gltf_assets.get(&drone_assets.0) else {
        return;
    };
    next_state.set(SimState::Running);
}
