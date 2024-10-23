use std::f32::consts::TAU;

use bevy::{
    asset::{AssetServer, Assets, Handle},
    color::Color,
    gltf::{Gltf, GltfNode},
    math::{EulerRot, Quat, Vec3},
    pbr::{DirectionalLight, DirectionalLightBundle},
    prelude::{
        default, BuildChildren, Camera3dBundle, Commands, NextState, Res, ResMut, Resource,
        SpatialBundle, Transform,
    },
    scene::SceneBundle,
};
use bevy_infinite_grid::InfiniteGridBundle;
use bevy_panorbit_camera::PanOrbitCamera;
use nalgebra::{Matrix3, Rotation3, Vector3};
use quad_sim::{
    arm::{Arm, MotorProps, MotorState, Propeller},
    rigid_body::{inv_cuboid_inertia_tensor, RigidBody},
    sample_curve::{SampleCurve, SamplePoint},
    Battery, BatteryProps, BatteryState, Drone, Gyro, Motor,
};

use crate::{
    ui::UiSimulationInfo, Controller, DroneComponent, FlightControllerComponent, SimContext,
    SimState,
};

// names of the propellers in the mesh
pub const PROP_BLADE_MESH_NAMES: [&str; 4] = [
    "prop_blade.001",
    "prop_blade.002",
    "prop_blade.003",
    "prop_blade.004",
];

#[derive(Resource, Clone)]
pub struct DroneAssets(Handle<Gltf>);

pub fn base_setup(mut commands: Commands, asset_server: Res<AssetServer>) {
    // Add a directional light to simulate the sun
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            color: Color::srgb(1.0, 1.0, 0.9), // Slightly warm sunlight
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
            // transform: Transform::from_translation(Vec3::new(0.0, 1.5, 5.0)),
            transform: Transform {
                translation: Vec3::new(0.0, 1.5, 5.0),
                ..Transform::IDENTITY
            },
            ..default()
        },
        PanOrbitCamera {
            yaw: Some(-TAU / 4.),
            ..Default::default()
        },
    ));

    // grid
    commands.spawn(InfiniteGridBundle::default());

    let drone_scene = asset_server.load("drone2.glb");
    let drone_assets = DroneAssets(drone_scene);
    commands.insert_resource(drone_assets.clone());
}

pub fn setup_drone(
    mut commands: Commands,
    drone_assets: Res<DroneAssets>,
    gltf_assets: Res<Assets<Gltf>>,
    gltf_node_assets: Res<Assets<GltfNode>>,
    mut next_state: ResMut<NextState<SimState>>,
) {
    // Wait until the scene is loaded
    if let Some(gltf) = gltf_assets.get(&drone_assets.0) {
        let flight_controller = FlightControllerComponent::new();
        flight_controller.init();
        let controller = Controller::default();
        // get the motor positions
        let motor_positions = PROP_BLADE_MESH_NAMES.map(|name| {
            let node_id = gltf.named_nodes[name].id();
            let prop_asset_node = gltf_node_assets.get(node_id).unwrap().clone();
            prop_asset_node.transform.translation.as_dvec3()
        });

        let arms = motor_positions.map(|position| {
            let motor = Motor {
                state: MotorState::default(),
                props: MotorProps {
                    position: Vector3::new(position.x, position.y, position.z),
                    motor_kv: 3200.,
                    motor_r: 0.13,
                    motor_io: 0.23,
                    ..Default::default()
                },
            };
            let propeller = Propeller {
                prop_inertia: 3.5e-07,
                prop_max_rpm: 36000.0,
                prop_a_factor: 7.43e-10,
                prop_torque_factor: 0.0056,
                prop_thrust_factor: Vector3::new(-5e-05, -0.0025, 4.75),
            };
            Arm {
                motor,
                propeller,
                ..Default::default()
            }
        });

        let drone = DroneComponent(Drone {
            arms,
            rigid_body: RigidBody {
                // random cuboid inv inertia tensor
                inv_tensor: Matrix3::from_diagonal(&Vector3::new(750., 5150.0, 750.0)),
                angular_velocity: Vector3::new(0.0, 0., 0.),
                mass: 0.2972,
                rotation: Rotation3::identity(), // stargin position
                frame_drag_area: Vector3::new(0.0082, 0.0077, 0.0082),
                frame_drag_constant: 1.45,
                linear_velocity: Vector3::zeros(),
                linear_velocity_dir: None,
                acceleration: Vector3::zeros(),
                position: Vector3::zeros(),
            },
            gyro: Gyro::default(),
            battery: Battery {
                state: BatteryState {
                    capacity: 850.,
                    ..Default::default()
                },
                props: BatteryProps {
                    bat_voltage_curve: SampleCurve::new(vec![
                        SamplePoint::new(-0.06, 4.4),
                        SamplePoint::new(0.0, 4.2),
                        SamplePoint::new(0.01, 4.05),
                        SamplePoint::new(0.04, 3.97),
                        SamplePoint::new(0.30, 3.82),
                        SamplePoint::new(0.40, 3.7),
                        SamplePoint::new(1.0, 3.49),
                        SamplePoint::new(1.01, 3.4),
                        SamplePoint::new(1.03, 3.3),
                        SamplePoint::new(1.06, 3.0),
                        SamplePoint::new(1.08, 0.0),
                    ]),
                    quad_bat_cell_count: 4,
                    quad_bat_capacity_charged: 850.,
                    quad_bat_capacity: 850.,
                    max_voltage_sag: 1.4,
                },
            },
            #[cfg(feature = "noise")]
            frame_charachteristics: FrameCharachteristics::default(),
        });

        commands
            .spawn((
                drone,
                SpatialBundle::default(),
                flight_controller,
                UiSimulationInfo::default(),
                SimContext::default(),
                controller,
            ))
            .with_children(|parent| {
                parent.spawn(SceneBundle {
                    scene: gltf.scenes[0].clone(),
                    ..Default::default()
                });
            });

        next_state.set(SimState::Running);
    }
}
