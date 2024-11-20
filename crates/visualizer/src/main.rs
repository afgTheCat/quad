mod controller;
mod game_loop;
mod setup;
mod ui;

use bevy::{
    app::{App, PluginGroup, Startup, Update},
    asset::{AssetServer, Assets, Handle},
    color::{palettes::css::RED, Color},
    ecs::query,
    gizmos::AppGizmoBuilder,
    gltf::{Gltf, GltfNode},
    math::{EulerRot, Quat, Vec3},
    pbr::{DirectionalLight, DirectionalLightBundle},
    prelude::{
        default, in_state, AppExtStates, Bundle, Camera3dBundle, Commands, Component, Deref,
        DerefMut, Entity, GizmoConfigGroup, Gizmos, IntoSystemConfigs, NextState, Query, Res,
        ResMut, Resource, SpatialBundle, States, Transform, With,
    },
    reflect::Reflect,
    scene::{Scene, SceneBundle},
    time::Time,
    window::{PresentMode, Window, WindowPlugin, WindowTheme},
    DefaultPlugins,
};
use bevy_egui::{egui::Window as EguiWindow, EguiContexts, EguiPlugin};
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use controller::{gamepad_input_events, gamepad_input_events_two};
use core::f64;
use egui_extras::{Column, TableBuilder};
use flight_controller::{
    controllers::bf_controller::BFController, Channels, FlightController, FlightControllerUpdate,
};
use game_loop::{debug_drone, ntb_mat3, ntb_vec3};
use nalgebra::{Matrix3, Rotation3, Vector3};
use setup::{base_setup, setup_drone, PROP_BLADE_MESH_NAMES};
#[cfg(feature = "noise")]
use simulator::FrameCharachteristics;
use simulator::{
    arm::{Arm, MotorProps, MotorState, Propeller},
    rigid_body::RigidBody,
    sample_curve::{SampleCurve, SamplePoint},
    Battery, BatteryProps, BatteryState, Drone, Gyro, Motor, SimulationDebugInfo,
};
use std::{sync::Arc, time::Duration};
use ui::update_ui;

#[derive(Clone, Component, Deref, DerefMut)]
struct DroneComponent(Drone);

#[derive(Component, Deref, DerefMut)]
struct FlightControllerComponent(Arc<dyn FlightController>);

impl FlightControllerComponent {
    fn new(fc: Arc<dyn FlightController>) -> Self {
        Self(fc)
    }
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
            dt: Duration::from_nanos(5000), // TODO: update this
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

#[derive(Component, Default, Deref, DerefMut)]
struct Controller(Channels);

impl Controller {
    pub fn to_channels(&self) -> Channels {
        self.0
    }
}

#[derive(Resource, Deref, DerefMut, Default)]
struct ControllerInput(Channels);

impl ControllerInput {
    pub fn to_channels(&self) -> Channels {
        self.0
    }
}

#[derive(Default, Reflect, GizmoConfigGroup)]
struct MyRoundGizmos {}

// Since we currently only support a single simulation, we should use a resource for the drone and
// all the auxulary information. In the future, if we include a multi drone setup/collisions and
// other things, it might make sense to have entities/components
// TODO: we probably want to move this to the simulation crate eventually
#[derive(Resource)]
struct Simulation {
    drone: Drone,
    flight_controller: Arc<dyn FlightController>,
    dt: Duration,
    time_accu: Duration, // the accumulated time between two steps + the correction from the
    ambient_temp: f64,
}

impl Simulation {
    fn simulate_delta(&mut self, delta: Duration, channels: Channels) -> SimulationDebugInfo {
        self.time_accu += delta;
        while self.time_accu > self.dt {
            let drone_state = self.drone.update(self.dt.as_secs_f64(), self.ambient_temp);
            let motor_input = self.flight_controller.update(FlightControllerUpdate {
                battery_update: drone_state.battery_update,
                gyro_update: drone_state.gyro_update,
                channels,
            });
            if let Some(motor_input) = motor_input {
                self.drone.set_motor_pwms(motor_input);
            }
            self.time_accu -= self.dt;
        }
        self.drone.debug_info()
    }
}

#[derive(Resource, Component, Clone)]
pub struct DroneAsset(Handle<Gltf>);

#[derive(Component, Clone)]
pub struct DroneComponentTwo;

// Set up the camera, light sources, the infinite grid, and start loading the drone scene. Loading
// glb objects in bevy is currently asyncronous and only when the scene is loaded should we
// initialize the flight controller and start the simulation
fn setup(mut commands: Commands, asset_server: Res<AssetServer>) {
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            color: Color::srgb(1.0, 1.0, 0.9),
            illuminance: 100000.0,
            shadows_enabled: true,
            ..Default::default()
        },
        transform: Transform {
            // Tilt the light to simulate the sun's angle (e.g., 45-degree angle)
            rotation: Quat::from_euler(EulerRot::XYZ, -std::f32::consts::FRAC_PI_4, 0.0, 0.0),
            ..Default::default()
        },
        ..Default::default()
    });

    commands.spawn((
        Camera3dBundle {
            transform: Transform {
                translation: Vec3::new(0.0, 1.5, 5.0),
                ..Transform::IDENTITY
            },
            ..default()
        },
        PanOrbitCamera {
            ..Default::default()
        },
    ));

    commands.spawn(InfiniteGridBundle::default());
    let drone_scene = asset_server.load("drone5.glb");
    let drone_assets = DroneAsset(drone_scene);
    commands.insert_resource(drone_assets.clone());
}

#[derive(Debug, Component)]
struct MarkerComponent;

fn setup_drone_two(
    mut commands: Commands,
    drone_assets: Res<DroneAsset>,
    gltf_assets: Res<Assets<Gltf>>,
    gltf_node_assets: Res<Assets<GltfNode>>,
    mut next_state: ResMut<NextState<SimState>>,
) {
    let Some(gltf) = gltf_assets.get(&drone_assets.0) else {
        return;
    };
    let flight_controller = Arc::new(BFController::new());
    flight_controller.init();
    let arms = PROP_BLADE_MESH_NAMES.map(|(name, motor_dir)| {
        let node_id = gltf.named_nodes[name].id();
        let prop_asset_node = gltf_node_assets.get(node_id).unwrap().clone();
        let position = prop_asset_node.transform.translation.as_dvec3();
        let motor = Motor {
            state: MotorState::default(),
            props: MotorProps {
                position: Vector3::new(position.x, position.y, position.z),
                motor_kv: 3200.,
                motor_r: 0.13,
                motor_io: 0.23,
                motor_dir,
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

    // Insert the new drone
    let drone = Drone {
        arms,
        rigid_body: RigidBody {
            // random cuboid inv inertia tensor
            inv_tensor: Matrix3::from_diagonal(&Vector3::new(750., 5150.0, 750.0)),
            angular_velocity: Vector3::new(0., 0., 0.),
            mass: 0.2972,
            rotation: Rotation3::identity(), // stargin position
            frame_drag_area: Vector3::new(0.0082, 0.0077, 0.0082),
            frame_drag_constant: 1.45,
            linear_velocity: Vector3::zeros(),
            linear_velocity_dir: None,
            acceleration: Vector3::zeros(),
            position: Vector3::zeros(),
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
    };

    // Insert the simulation resource
    commands.insert_resource(Simulation {
        drone,
        flight_controller,
        time_accu: Duration::default(),
        dt: Duration::from_nanos(5000), // TODO: update this
        ambient_temp: 25.,
    });

    // Insert the simulation debug info
    commands.insert_resource(DebugUiInfo::default());

    // Insert the controller input
    commands.insert_resource(ControllerInput::default());

    // Insert the scene bundle
    commands.spawn((
        SceneBundle {
            scene: gltf.scenes[0].clone(),
            ..Default::default()
        },
        MarkerComponent,
    ));

    // Set next state
    next_state.set(SimState::Running);
}

#[derive(Resource, Deref, Default)]
struct DebugUiInfo(pub SimulationDebugInfo);

fn sim_loop(
    mut gizmos: Gizmos,
    timer: Res<Time>,
    mut ui_info: ResMut<DebugUiInfo>,
    mut simulation: ResMut<Simulation>,
    controller_input: Res<ControllerInput>,
    mut camera_query: Query<&mut PanOrbitCamera>,
    mut scene_query: Query<(&mut Transform, &Handle<Scene>)>,
    query: Query<Entity, With<MarkerComponent>>,
) {
    match query.iter().next() {
        Some(entity) => println!("Entity {:?} still exists!", entity),
        _ => println!("Entitiy does not exists"),
    }
    let (mut tranform, _) = scene_query.single_mut();
    let mut camera = camera_query.single_mut();
    let debug_info = simulation.simulate_delta(timer.delta(), controller_input.to_channels());

    let drone_translation = ntb_vec3(debug_info.position);
    let drone_rotation = Quat::from_mat3(&ntb_mat3(debug_info.rotation));
    tranform.translation = drone_translation;
    tranform.rotation = drone_rotation;

    let down_dir = debug_info.rotation * Vector3::new(0., -1., -0.);
    for motor_index in 0..4 {
        let motor_pos = drone_translation
            + ntb_vec3(debug_info.rotation * simulation.drone.arms[motor_index].motor_pos());
        gizmos.arrow(
            motor_pos,
            motor_pos + ntb_vec3(down_dir * simulation.drone.arms[motor_index].thrust()),
            RED,
        );
    }

    *ui_info = DebugUiInfo(debug_info);
    camera.target_focus = drone_translation;
}

fn update_ui_two(mut ctx: EguiContexts, ui_sim_info: Res<DebugUiInfo>) {
    EguiWindow::new("Simulation info").show(ctx.ctx_mut(), |ui| {
        TableBuilder::new(ui)
            .column(Column::auto().resizable(true))
            .column(Column::remainder())
            .header(20.0, |mut header| {
                header.col(|ui| {
                    ui.heading("Name");
                });
                header.col(|ui| {
                    ui.heading("Data");
                });
            })
            .body(|mut body| {
                // Rust macros are hygenic, so we need to declare the macro in the scope where body
                // is already defined
                macro_rules! display_debug_data {
                    ($column_template:literal, $column_value:expr, $column_data_length:literal) => {
                        for i in 0..$column_data_length {
                            let column_name = format!($column_template, i);
                            display_debug_data!(column_name, $column_value[i]);
                        }
                    };

                    ($column_name:expr, $column_value:expr) => {
                        body.row(30.0, |mut row| {
                            row.col(|ui| {
                                ui.label($column_name);
                            });
                            row.col(|ui| {
                                ui.label(format!("{:?}", $column_value));
                            });
                        });
                    };
                }
                // Display all the data that we want to show
                // TODO: maybe readd this
                // display_debug_data!("Determinant", ui_sim_info.rotation.determinant());
                display_debug_data!("Rotation matrix", ui_sim_info.rotation);
                display_debug_data!("Velocity", ui_sim_info.linear_velocity);
                display_debug_data!("Acceleration", ui_sim_info.acceleration);
                display_debug_data!("Angular velocity", ui_sim_info.angular_velocity);
                display_debug_data!("Motor thrust {}", ui_sim_info.thrusts, 4);
                display_debug_data!("Motor rpm {}", ui_sim_info.rpms, 4);
                display_debug_data!("Motor pwm {}", ui_sim_info.pwms, 4);
                display_debug_data!("Bat voltage", ui_sim_info.bat_voltage);
                display_debug_data!("Bat voltage sag", ui_sim_info.bat_voltage_sag);
            });
    });
}

fn build_app_two() -> App {
    let mut app = App::new();
    let default_plugin = DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "Sim".into(),
            name: Some("bevy.app".into()),
            resolution: (2560., 1440.).into(),
            present_mode: PresentMode::AutoVsync,
            fit_canvas_to_parent: true,
            prevent_default_event_handling: false,
            window_theme: Some(WindowTheme::Dark),
            enabled_buttons: bevy::window::EnabledButtons {
                maximize: false,
                ..Default::default()
            },
            ..default()
        }),
        ..default()
    });
    app.add_plugins(default_plugin)
        .add_plugins(EguiPlugin)
        .add_plugins(PanOrbitCameraPlugin)
        .insert_state(SimState::Loading)
        .init_gizmo_group::<MyRoundGizmos>()
        .add_plugins(InfiniteGridPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, setup_drone_two.run_if(in_state(SimState::Loading)))
        .add_systems(Update, sim_loop.run_if(in_state(SimState::Running)))
        .add_systems(
            Update,
            gamepad_input_events_two.run_if(in_state(SimState::Running)),
        )
        .add_systems(Update, update_ui_two.run_if(in_state(SimState::Running)));
    app
}

fn build_app() -> App {
    let mut app = App::new();
    let default_plugin = DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "Sim".into(),
            name: Some("bevy.app".into()),
            resolution: (2560., 1440.).into(),
            present_mode: PresentMode::AutoVsync,
            fit_canvas_to_parent: true,
            prevent_default_event_handling: false,
            window_theme: Some(WindowTheme::Dark),
            enabled_buttons: bevy::window::EnabledButtons {
                maximize: false,
                ..Default::default()
            },
            ..default()
        }),
        ..default()
    });
    app.add_plugins(default_plugin)
        .add_plugins(EguiPlugin)
        .add_plugins(PanOrbitCameraPlugin)
        .insert_state(SimState::Loading)
        .init_gizmo_group::<MyRoundGizmos>()
        .add_plugins(InfiniteGridPlugin)
        .add_systems(Startup, base_setup)
        .add_systems(Update, setup_drone.run_if(in_state(SimState::Loading)))
        .add_systems(Update, debug_drone.run_if(in_state(SimState::Running)))
        .add_systems(
            Update,
            gamepad_input_events.run_if(in_state(SimState::Running)),
        )
        .add_systems(Update, update_ui.run_if(in_state(SimState::Running)));
    app
}

fn main() {
    let mut app = build_app_two();
    app.run();
}
