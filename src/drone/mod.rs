use std::time::Duration;

use bevy::{
    asset::{AssetServer, Assets, Handle},
    color::{palettes::css::RED, Color},
    gltf::{Gltf, GltfMesh, GltfNode},
    math::{EulerRot, Quat, Vec3, VectorSpace},
    pbr::{DirectionalLight, DirectionalLightBundle, PbrBundle},
    prelude::{
        default, Camera3dBundle, Commands, Component, Gizmos, Mesh, NextState, Query, Res, ResMut,
        Resource, Transform,
    },
    time::Time,
};
use bevy_infinite_grid::InfiniteGridBundle;
use bevy_panorbit_camera::PanOrbitCamera;
use body::Drone;
use motor::Motor;
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

const PROP_BLADES: [&str; 4] = [
    "prop_blade.001",
    "prop_blade.002",
    "prop_blade.003",
    "prop_blade.004",
];

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

        let prop_nodes = gltf
            .named_nodes
            .keys()
            .filter(|key| PROP_BLADES.contains(&key.as_ref()))
            .map(|key| gltf.named_nodes[key].id());

        // add props
        for prop_node_id in prop_nodes {
            let prop_asset_node = gltf_node_assets.get(prop_node_id).unwrap().clone();
            let mesh_id = prop_asset_node.mesh.unwrap().id();
            let mesh = gltf_mesh_assets.get(mesh_id).unwrap();
            let primitive = mesh.primitives[0].clone();

            // props are only one primitive
            commands.spawn((
                PbrBundle {
                    mesh: primitive.mesh.clone(),
                    material: primitive.material.unwrap(),
                    transform: prop_asset_node.transform,
                    ..Default::default()
                },
                Motor::default(),
            ));
        }

        // let drone_body_node = gltf.named_nodes["drone_body"].clone();
        // let drone_body_asset_node = gltf_node_assets.get(drone_body_node.id()).unwrap().clone();
        // let drone_body_asset_id = drone_body_asset_node.mesh.unwrap().id();
        // let drone_body_mesh = gltf_mesh_assets.get(drone_body_asset_id).unwrap();
        // for primitive in &drone_body_mesh.primitives {
        //     commands.spawn(PbrBundle {
        //         mesh: primitive.mesh.clone(),
        //         material: primitive.material.clone().unwrap(),
        //         transform: drone_body_asset_node.transform, // Apply the same transform to all primitives
        //         ..Default::default()
        //     });
        // }
    }
}

pub fn debug_drone(mut gizmos: Gizmos, mut query: Query<(&Transform, &Motor)>) {
    for (transform, motor) in query.iter() {
        println!("{}", transform.translation);
        gizmos.arrow(Vec3::ZERO, transform.translation, RED);
    }
}

// let meshes = gltf.named_meshes.keys().collect_vec();
// println!("meshes: {:?}", meshes);
//
// let meshes = gltf.named_materials.keys().collect_vec();
// println!("named materials: {:?}", meshes);
//
// let nodes = gltf.named_nodes.keys().collect_vec();
// println!("named nodes: {:?}", nodes);
//
// let node_ids = gltf.named_nodes.values().map(|v| v.id()).collect_vec();
// let asd = gltf_nodes.get(node_ids[0]);
// println!("{:?}", asd);
