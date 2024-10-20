mod ui;
use bevy::{
    app::{App, PluginGroup, Startup, Update},
    asset::{AssetServer, Assets, Handle},
    color::Color,
    gizmos::AppGizmoBuilder,
    gltf::{Gltf, GltfNode},
    math::{DMat3, DVec3, DVec4, EulerRot, Mat3, Quat, Vec3},
    pbr::{DirectionalLight, DirectionalLightBundle},
    prelude::{
        default, in_state, AppExtStates, BuildChildren, Camera3dBundle, Commands, Component,
        GizmoConfigGroup, IntoSystemConfigs, NextState, Query, Res, ResMut, Resource,
        SpatialBundle, States, Transform,
    },
    reflect::Reflect,
    scene::SceneBundle,
    time::Time,
    window::{PresentMode, Window, WindowPlugin, WindowTheme},
    DefaultPlugins,
};
use bevy_egui::EguiPlugin;
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use controller::{
    controllers::bf_controller::BFController, BatteryUpdate, Channels, FlightController,
    FlightControllerUpdate, GyroUpdate, MotorInput,
};
use core::f64;
use nalgebra::{Matrix3, Vector3, Vector4};

#[cfg(feature = "noise")]
use quad_sim::FrameCharachteristics;
use quad_sim::{
    arm::{Arm, MotorProps, MotorState, Propeller},
    // controller::Model,
    rigid_body::{inv_cuboid_inertia_tensor, RigidBody},
    sample_curve::{SampleCurve, SamplePoint},
    Battery,
    BatteryProps,
    BatteryState,
    Drone,
    Gyro,
    Motor,
};
use std::{sync::Arc, time::Duration};
use ui::{update_ui, UiSimulationInfo};

#[derive(Clone, Component)]
pub struct DroneComponent(Drone);

impl DroneComponent {
    fn set_motor_pwms(&mut self, input: MotorInput) {
        self.0.set_motor_pwms(input)
    }

    fn update_gyro(&mut self, dt: f64) {
        self.0.update_gyro(dt)
    }

    fn update_physics(&mut self, dt: f64, ambient_temp: f64) {
        self.0.update_physics(dt, ambient_temp)
    }

    fn battery_update(&self) -> BatteryUpdate {
        self.0.battery.battery_update()
    }

    fn gyro_update(&self) -> GyroUpdate {
        self.0.gyro.gyro_update()
    }
}

#[derive(Component)]
pub struct FlightControllerComponent {
    fc: Arc<dyn FlightController>,
}

impl FlightControllerComponent {
    fn init(&self) {
        self.fc.init()
    }

    fn update(&self, update: FlightControllerUpdate) -> MotorInput {
        self.fc.update(update)
    }
}

// names of the propellers in the mesh
pub const PROP_BLADE_MESH_NAMES: [&str; 4] = [
    "prop_blade.001",
    "prop_blade.002",
    "prop_blade.003",
    "prop_blade.004",
];

fn ntb_vec3(vec: Vector3<f64>) -> Vec3 {
    Vec3::new(vec[0] as f32, vec[1] as f32, vec[2] as f32)
}

fn ntb_dvec3(vec: Vector3<f64>) -> DVec3 {
    DVec3::new(vec[0], vec[1], vec[2])
}

fn ntb_dvec4(vec: Vector4<f64>) -> DVec4 {
    DVec4::new(vec[0], vec[1], vec[2], vec[3])
}

fn ntb_mat3(matrix: Matrix3<f64>) -> Mat3 {
    Mat3::from_cols(
        Vec3::from_slice(
            &matrix
                .column(0)
                .iter()
                .map(|x| *x as f32)
                .collect::<Vec<_>>(),
        ),
        Vec3::from_slice(
            &matrix
                .column(1)
                .iter()
                .map(|x| *x as f32)
                .collect::<Vec<_>>(),
        ),
        Vec3::from_slice(
            &matrix
                .column(2)
                .iter()
                .map(|x| *x as f32)
                .collect::<Vec<_>>(),
        ),
    )
}

fn ntb_dmat3(matrix: Matrix3<f64>) -> DMat3 {
    DMat3::from_cols(
        DVec3::from_slice(&matrix.column(0).iter().map(|x| *x).collect::<Vec<_>>()),
        DVec3::from_slice(&matrix.column(1).iter().map(|x| *x).collect::<Vec<_>>()),
        DVec3::from_slice(&matrix.column(2).iter().map(|x| *x).collect::<Vec<_>>()),
    )
}

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
            dt: Duration::from_nanos(500),
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
                inv_tensor: inv_cuboid_inertia_tensor(Vector3::new(750., -5150.0, 750.0)),
                angular_velocity: Vector3::new(0., 0., 0.),
                mass: 0.2972,
                rotation: Matrix3::identity(), // stargin position
                frame_drag_area: Vector3::new(0.0082, 0.0077, 0.0082),
                frame_drag_constant: 1.45,
                ..Default::default()
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
                FlightControllerComponent {
                    fc: Arc::new(BFController),
                },
                UiSimulationInfo::default(),
                SimContext::default(),
            ))
            .with_children(|parent| {
                parent.spawn(SceneBundle {
                    scene: gltf.scenes[0].clone(),
                    ..Default::default()
                });
            });
    }
}

// The thing is, I don't want this to be a generic thing
pub fn debug_drone(
    mut drone_query: Query<(
        &mut Transform,
        &mut DroneComponent,
        &mut FlightControllerComponent,
        &mut UiSimulationInfo,
        &mut SimContext,
    )>,
    mut camera_query: Query<&mut PanOrbitCamera>,
    timer: Res<Time>,
) {
    let (mut drone_transform, mut drone, mut controller, mut ui_simulation_info, mut sim_context) =
        drone_query.single_mut();
    let mut camera = camera_query.single_mut();

    sim_context.time_accu += timer.delta();
    while sim_context.step_context() {
        drone.update_gyro(sim_context.dt.as_secs_f64());
        drone.update_physics(sim_context.dt.as_secs_f64(), sim_context.ambient_temp);
        let battery_update = drone.battery_update();
        let gyro_update = drone.gyro_update();
        let fc_input = FlightControllerUpdate {
            gyro_update,
            battery_update,
            channels: Channels {
                throttle: 0.5,
                yaw: 0.5,
                pitch: 0.5,
                roll: 0.5,
            },
        };
        let pwms = controller.update(fc_input);
        drone.set_motor_pwms(pwms);
    }

    let drone_translation = ntb_vec3(drone.0.rigid_body.position);
    drone_transform.translation = drone_translation;
    drone_transform.rotation = Quat::from_mat3(&ntb_mat3(drone.0.rigid_body.rotation));
    ui_simulation_info.update_state(
        ntb_dmat3(drone.0.rigid_body.rotation),
        ntb_dvec3(drone.0.rigid_body.position),
        ntb_dvec3(drone.0.rigid_body.linear_velocity),
        ntb_dvec3(drone.0.rigid_body.acceleration),
        ntb_dvec3(drone.0.rigid_body.angular_velocity),
        ntb_dvec4(drone.0.thrusts()),
        ntb_dvec4(drone.0.rpms()),
        drone.0.battery.state.bat_voltage,
        drone.0.battery.state.bat_voltage_sag,
    );
    camera.target_focus = drone_translation;
}

#[derive(Default, Reflect, GizmoConfigGroup)]
struct MyRoundGizmos {}

pub fn build_app() -> App {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "Sim".into(),
            name: Some("bevy.app".into()),
            resolution: (2560., 1440.).into(),
            present_mode: PresentMode::AutoVsync,
            // Tells Wasm to resize the window according to the available canvas
            fit_canvas_to_parent: true,
            // Tells Wasm not to override default event handling, like F5, Ctrl+R etc.
            prevent_default_event_handling: false,
            window_theme: Some(WindowTheme::Dark),
            enabled_buttons: bevy::window::EnabledButtons {
                maximize: false,
                ..Default::default()
            },
            ..default()
        }),
        ..default()
    }))
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
