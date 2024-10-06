use arm::Arm;
use bevy::{
    asset::{AssetServer, Assets, Handle},
    color::{palettes::css::RED, Color},
    gltf::{Gltf, GltfMesh, GltfNode},
    math::{EulerRot, Quat, Vec3, VectorSpace},
    pbr::{DirectionalLight, DirectionalLightBundle, PbrBundle},
    prelude::{
        default, BuildChildren, Camera3dBundle, Commands, Component, Gizmos, Mesh, NextState,
        Query, Res, ResMut, Resource, SpatialBundle, Transform,
    },
    scene::{Scene, SceneBundle},
    time::Time,
};
use bevy_asset_loader::prelude::*;
use bevy_infinite_grid::InfiniteGridBundle;
use bevy_panorbit_camera::PanOrbitCamera;
use body::Body;
use itertools::Itertools;
use motor::{Motor, MotorProps, MotorState};
use state_packet::StatePacket;
use std::time::Duration;

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

const PROP_BLADES: [&str; 4] = [
    "prop_blade.001",
    "prop_blade.002",
    "prop_blade.003",
    "prop_blade.004",
];

// #[derive(AssetCollection, Resource)]
// pub struct Models {
//     #[asset(path = "")]
//     pub drone: Handle<Scene>,
// }

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
    mut query: Query<(&mut Transform, &mut DroneContext, &mut Model, &mut Body)>,
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

pub fn base_setup(mut commands: Commands, asset_server: Res<AssetServer>) {
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
    mut commands: Commands,
    drone_assets: Res<DroneAssets>,
    gltf_assets: Res<Assets<Gltf>>,
    gltf_node_assets: Res<Assets<GltfNode>>,
    gltf_mesh_assets: Res<Assets<GltfMesh>>,
    mut next_state: ResMut<NextState<SimState>>,
) {
    // Wait until the scene is loaded
    if let Some(gltf) = gltf_assets.get(&drone_assets.0) {
        next_state.set(SimState::Running);

        // get the motor positions
        let motor_positions = PROP_BLADES.map(|name| {
            let node_id = gltf.named_nodes[name].id();
            let prop_asset_node = gltf_node_assets.get(node_id).unwrap().clone();
            prop_asset_node.transform.translation.as_dvec3()
        });

        let arms = motor_positions.map(|position| {
            let motor = Motor {
                state: MotorState {
                    position,
                    ..Default::default()
                },
                props: MotorProps::default(),
            };
            Arm {
                motor,
                ..Default::default()
            }
        });

        let drone = Body {
            arms,
            ..Default::default()
        };
        commands
            .spawn((drone, SpatialBundle::default()))
            .with_children(|parent| {
                parent.spawn(SceneBundle {
                    scene: gltf.scenes[0].clone(),
                    ..Default::default()
                });
            });
    }
}

pub fn debug_drone(mut gizmos: Gizmos, mut query: Query<(&Transform, &Body)>) {
    let (_, body) = query.single();
    for arm in body.arms.iter() {
        gizmos.arrow(Vec3::ZERO, arm.motor_pos().as_vec3(), RED);
    }
    // for (transform, motor) in query.iter() {
    //     gizmos.arrow(Vec3::ZERO, transform.translation, RED);
    // }
}
