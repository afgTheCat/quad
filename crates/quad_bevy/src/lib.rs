mod ui;

use bevy::{
    app::{App, Startup, Update},
    asset::{AssetServer, Assets, Handle},
    color::Color,
    gizmos::AppGizmoBuilder,
    gltf::{Gltf, GltfNode},
    math::{EulerRot, Quat, Vec3},
    pbr::{DirectionalLight, DirectionalLightBundle},
    prelude::{
        default, in_state, AppExtStates, BuildChildren, Camera3dBundle, Commands, Component,
        GizmoConfigGroup, IntoSystemConfigs, NextState, Query, Res, ResMut, Resource,
        SpatialBundle, States, Transform,
    },
    reflect::Reflect,
    scene::SceneBundle,
    time::Time,
    DefaultPlugins,
};
use bevy_egui::EguiPlugin;
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use quad_sim::{
    arm::{Arm, MotorProps, MotorState},
    controller::Model,
    rigid_body::{inv_cuboid_inertia_tensor, RigidBody},
    sample_curve::{SampleCurve, SamplePoint},
    Battery, BatteryProps, BatteryState, Drone, Gyro, Motor,
};
use std::time::Duration;
use ui::{update_ui, UiSimulationInfo};

// names of the propellers in the mesh
pub const PROP_BLADE_MESH_NAMES: [&str; 4] = [
    "prop_blade.001",
    "prop_blade.002",
    "prop_blade.003",
    "prop_blade.004",
];

#[derive(States, Clone, Copy, Default, Eq, PartialEq, Hash, Debug)]
pub enum SimState {
    #[default]
    Loading,
    Running,
}

#[derive(Component)]
pub struct SimContext {
    pub dt: Duration,
    pub time_accu: Duration, // the accumulated time between two steps + the correction from the
    pub ambient_temp: f64,
    pub dialation: f64,
}

impl Default for SimContext {
    fn default() -> Self {
        Self {
            dt: Duration::from_nanos(1000),
            time_accu: Duration::default(),
            ambient_temp: 25.,
            dialation: 1.,
        }
    }
}

impl SimContext {
    fn step_context(&mut self) -> bool {
        if self.time_accu > self.dt {
            self.time_accu -= self.dt;
            true
        } else {
            false
        }
    }
}

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
            transform: Transform::from_translation(Vec3::new(0.0, 1.5, 5.0)),
            ..default()
        },
        PanOrbitCamera::default(),
    ));

    // grid
    commands.spawn(InfiniteGridBundle::default());

    // sim context
    commands.spawn(SimContext::default());

    let drone_scene = asset_server.load("drone.glb");
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
        next_state.set(SimState::Running);

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
                    position,
                    ..Default::default()
                },
            };
            Arm {
                motor,
                ..Default::default()
            }
        });

        let drone = Drone {
            arms,
            rigid_body: RigidBody {
                // random cuboid inv inertia tensor
                inv_tensor: inv_cuboid_inertia_tensor(Vec3::new(0.1, 0.1, 0.1)),
                mass: 0.2,
                ..Default::default()
            },
            gyro: Gyro::default(),
            battery: Battery {
                state: BatteryState::default(),
                props: BatteryProps {
                    full_capacity: 1.,
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
                    quad_bat_cell_count: 6.,
                    quad_bat_capacity_charged: 10000.,
                    max_voltage_sag: 0.,
                },
            },
            #[cfg(feature = "noise")]
            frame_charachteristics: FrameCharachteristics::default(),
        };
        commands
            .spawn((
                drone,
                SpatialBundle::default(),
                Model::default(),
                UiSimulationInfo::default(),
            ))
            .with_children(|parent| {
                parent.spawn(SceneBundle {
                    scene: gltf.scenes[0].clone(),
                    ..Default::default()
                });
            });
    }
}

pub fn debug_drone(
    mut drone_query: Query<(
        &mut Transform,
        &mut Drone,
        &mut Model,
        &mut UiSimulationInfo,
    )>,
    mut context_query: Query<&mut SimContext>,
    timer: Res<Time>,
) {
    let (mut transform, mut drone, mut controller, mut ui_simulation_info) =
        drone_query.single_mut();
    let mut sim_context = context_query.single_mut();

    sim_context.time_accu += timer.delta();
    while sim_context.step_context() {
        drone.update_gyro(sim_context.dt.as_secs_f64());
        drone.update_physics(sim_context.dt.as_secs_f64(), sim_context.ambient_temp);
        let pwms = controller.update(&drone);
        drone.set_motor_pwms(pwms.pwms());
    }

    transform.translation = drone.rigid_body.position.as_vec3();
    transform.rotation = Quat::from_mat3(&drone.rigid_body.rotation.as_mat3());
    ui_simulation_info.update_state(
        drone.rigid_body.rotation,
        drone.rigid_body.position,
        drone.rigid_body.linear_velocity,
        drone.rigid_body.acceleration,
    );
}

#[derive(Default, Reflect, GizmoConfigGroup)]
struct MyRoundGizmos {}

pub fn build_app() -> App {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin)
        .add_plugins(PanOrbitCameraPlugin)
        .insert_state(SimState::Loading)
        .init_gizmo_group::<MyRoundGizmos>()
        .add_plugins(InfiniteGridPlugin)
        .add_systems(Startup, base_setup)
        .add_systems(Update, setup_drone.run_if(in_state(SimState::Loading)))
        .add_systems(Update, debug_drone.run_if(in_state(SimState::Running)))
        .add_systems(Update, update_ui.run_if(in_state(SimState::Running)));
    app
}
