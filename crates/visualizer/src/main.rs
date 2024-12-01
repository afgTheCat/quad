//! The `simulation` crate provides a small bevy application that can be used to test how different
//! contfigurations may affect the drone and it's behaviour. This software is super early in
//! development, expect a lot of changes, including to how configurations are stored, what drone
//! meshes we want to use etc.
use bevy::{
    app::{App, PluginGroup, Startup, Update},
    asset::{AssetServer, Assets, Handle},
    color::{palettes::css::RED, Color},
    gltf::{Gltf, GltfNode},
    input::{gamepad::GamepadEvent, keyboard::KeyboardInput, ButtonState},
    math::{EulerRot, Mat3, Quat, Vec3},
    pbr::{DirectionalLight, DirectionalLightBundle},
    prelude::{
        default, in_state, AppExtStates, Camera3dBundle, Commands, Deref, DerefMut, EventReader,
        GamepadAxisType, Gizmos, IntoSystemConfigs, KeyCode, NextState, Query, Res, ResMut,
        Resource, States, Transform,
    },
    scene::{Scene, SceneBundle},
    time::Time,
    window::{PresentMode, Window, WindowPlugin, WindowTheme},
    DefaultPlugins,
};
use bevy_egui::{egui::Window as EguiWindow, EguiContexts, EguiPlugin};
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use core::f64;
use egui_extras::{Column, TableBuilder};
use flight_controller::{
    controllers::bf_controller::BFController, Channels, FlightController, FlightControllerUpdate,
};
use nalgebra::{AbstractRotation, Matrix3, Rotation3, UnitQuaternion, Vector3};
#[cfg(feature = "noise")]
use simulator::FrameCharachteristics;
use simulator::{
    low_pass_filter::LowPassFilter,
    sample_curve::{SampleCurve, SamplePoint},
    BatteryModel, BatteryStateTwo, DroneFrameStateTwo, DroneModel, GyroModel, GyroStateTwo,
    RotorModel, RotorStateTwo, RotorsStateTwo, SimulationDebugInfo, SimulationFrame, SimulationTwo,
};
use std::{sync::Arc, time::Duration};

// TODO: we should not rely on the mesh names for the simulation
const PROP_BLADE_MESH_NAMES: [(&str, f64); 4] = [
    ("prop_blade.001", -1.),
    ("prop_blade.002", 1.),
    ("prop_blade.003", 1.),
    ("prop_blade.004", -1.),
];

/// The simulation has two states: when the drone mesh is loading, and when the simulation is
/// running. In the future we would like to implement a stopped state as well in order to enable
/// simulation resetting
#[derive(States, Clone, Copy, Default, Eq, PartialEq, Hash, Debug)]
enum SimState {
    #[default]
    Loading,
    Running,
}

/// A helper function to transform an nalgebra::Vector3 to a Vec3 used by bevy
fn ntb_vec3(vec: Vector3<f64>) -> Vec3 {
    Vec3::new(vec[0] as f32, vec[1] as f32, vec[2] as f32)
}

/// A helper function to transform an nalgebra::Rotation3 to a Mat3 used by bevy
fn ntb_mat3(matrix: Rotation3<f64>) -> Mat3 {
    Mat3::from_cols(
        Vec3::from_slice(
            &matrix
                .matrix()
                .column(0)
                .iter()
                .map(|x| *x as f32)
                .collect::<Vec<_>>(),
        ),
        Vec3::from_slice(
            &matrix
                .matrix()
                .column(1)
                .iter()
                .map(|x| *x as f32)
                .collect::<Vec<_>>(),
        ),
        Vec3::from_slice(
            &matrix
                .matrix()
                .column(2)
                .iter()
                .map(|x| *x as f32)
                .collect::<Vec<_>>(),
        ),
    )
}

/// Stores controller inputs in the `PlayerControllerInput` resource. This is input to the simulation
/// as the setpoints for the controller. We need to store it as it is not guaranteed that a controller
/// input will be sent on each frame.
fn store_controller_input(
    mut evr_gamepad: EventReader<GamepadEvent>,
    mut evr_kbd: EventReader<KeyboardInput>, // TODO: just for debugging
    mut controller_input: ResMut<PlayerControllerInput>,
) {
    for ev in evr_gamepad.read() {
        let &GamepadEvent::Axis(ax) = &ev else {
            continue;
        };

        let ax_val = ax.value as f64;

        match ax.axis_type {
            GamepadAxisType::LeftZ => controller_input.throttle = ax_val,
            GamepadAxisType::RightStickX => {
                controller_input.yaw = if ax_val > -0.96 { ax_val } else { -1. }
            }
            GamepadAxisType::LeftStickX => controller_input.roll = ax_val,
            GamepadAxisType::LeftStickY => controller_input.pitch = -ax_val,
            _ => {}
        }
    }

    for ev in evr_kbd.read() {
        if let (KeyCode::Space, ButtonState::Pressed) = (ev.key_code, ev.state) {
            controller_input.throttle = 1.;
        }

        if let (KeyCode::Space, ButtonState::Released) = (ev.key_code, ev.state) {
            controller_input.throttle = -1.;
        }

        if let (KeyCode::ArrowLeft, ButtonState::Pressed) = (ev.key_code, ev.state) {
            controller_input.yaw -= 0.01;
        }

        if let (KeyCode::ArrowRight, ButtonState::Pressed) = (ev.key_code, ev.state) {
            controller_input.yaw += 0.01;
        }
    }
}

/// Acts as storage for the controller inputs. Controller inputs are used as setpoints for the
/// controller. We are storing them since it's not guaranteed that a new inpout will be sent on
/// each frame.
/// TODO: do we even need this? I assume that betaflight will handle storing the inputs.
#[derive(Resource, Deref, DerefMut, Default)]
struct PlayerControllerInput(Channels);

impl PlayerControllerInput {
    fn to_channels(&self) -> Channels {
        self.0
    }
}

// Since we currently only support a single simulation, we should use a resource for the drone and
// all the auxulary information. In the future, if we include a multi drone setup/collisions and
// other things, it might make sense to have entities/components
// TODO: we probably want to move this to the simulation crate eventually
#[derive(Resource)]
struct Simulation {
    drone: SimulationTwo,
    flight_controller: Arc<dyn FlightController>,
    dt: Duration,
    time_accu: Duration, // the accumulated time between two steps + the correction from the
}

impl Simulation {
    /// Given a duration (typically 100ms between frames), runs the simulation until the time
    /// accumlator is less then the simulation's dt. It will also try to
    fn simulate_delta(&mut self, delta: Duration, channels: Channels) -> SimulationDebugInfo {
        self.time_accu += delta;
        while self.time_accu > self.dt {
            let drone_state = self.drone.update();
            let motor_input = self.flight_controller.update(FlightControllerUpdate {
                battery_update: drone_state.battery_update,
                gyro_update: drone_state.gyro_update,
                channels,
            });
            if let Some(motor_input) = motor_input {
                // TODO: haha this fixes a lot of things
                // println!(
                //     "motor input: {:?}, angular_velocity: {:?}",
                //     motor_input, drone_state.gyro_update.angular_velocity
                // );
                self.drone.set_motor_pwms(motor_input);
            }
            self.time_accu -= self.dt;
        }
        self.drone.debug_info()
    }
}

/// Holds all the relevant data that the we wish to show.
#[derive(Resource, Deref, Default)]
struct DebugUiContent(SimulationDebugInfo);

/// The drone asset wrapper. It is used to query for the drone asset within the gltf assets.
#[derive(Resource, Clone, Deref)]
struct DroneAsset(Handle<Gltf>);

/// Set up the camera, light sources, the infinite grid, and start loading the drone scene. Loading
/// glb objects in bevy is currently asyncronous and only when the scene is loaded should we
/// initialize the flight controller and start the simulation
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
    let drone_asset = DroneAsset(drone_scene);
    commands.insert_resource(drone_asset.clone());
}

/// When the drone asset is loaded, sets up the `Simulation` and sets the new `SimState` to
/// `SimState::Running`. It will also handle setting up the `DebugUiContent`, the
/// `PlayerControllerInput` and the spawns the scene.
fn setup_drone(
    mut commands: Commands,
    drone_asset: Res<DroneAsset>,
    gltf_assets: Res<Assets<Gltf>>,
    gltf_node_assets: Res<Assets<GltfNode>>,
    mut next_state: ResMut<NextState<SimState>>,
) {
    let Some(gltf) = gltf_assets.get(&drone_asset.0) else {
        return;
    };
    let flight_controller = Arc::new(BFController::new());
    flight_controller.init();

    let rotors_state = RotorsStateTwo(PROP_BLADE_MESH_NAMES.map(|(name, rotor_dir)| {
        let node_id = gltf.named_nodes[name].id();
        let prop_asset_node = gltf_node_assets.get(node_id).unwrap().clone();
        let position = prop_asset_node.transform.translation.as_dvec3();
        // guess its fine
        RotorStateTwo {
            current: 0.,
            rpm: 0.,
            motor_torque: 0.,
            effective_thrust: 0.,
            pwm: 0.,
            rotor_dir,
            motor_pos: Vector3::new(position.x, position.y, position.z),
        }
    }));

    let battery_state = BatteryStateTwo {
        capacity: 850.,
        bat_voltage: 4.2,
        bat_voltage_sag: 4.2,
        amperage: 0.,
        m_ah_drawn: 0.,
    };

    let drone_state = DroneFrameStateTwo {
        position: Vector3::zeros(),
        rotation: Rotation3::identity(), // stargin position
        linear_velocity: Vector3::zeros(),
        angular_velocity: Vector3::new(0., 0., 0.),
        acceleration: Vector3::zeros(),
    };

    let gyro_state = GyroStateTwo {
        rotation: UnitQuaternion::identity(),
        acceleration: Vector3::zeros(),
        angular_velocity: Vector3::zeros(),
    };

    let initial_frame = SimulationFrame {
        battery_state,
        rotors_state,
        drone_state,
        gyro_state,
    };

    let bat_voltage_curve = SampleCurve::new(vec![
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
    ]);

    let battery_model = BatteryModel {
        quad_bat_capacity: 850.,
        bat_voltage_curve,
        quad_bat_cell_count: 4,
        quad_bat_capacity_charged: 850.,
        max_voltage_sag: 1.4,
    };

    let rotor_model = RotorModel {
        prop_max_rpm: 36000.0,
        pwm_low_pass_filter: [
            LowPassFilter::default(),
            LowPassFilter::default(),
            LowPassFilter::default(),
            LowPassFilter::default(),
        ],
        motor_kv: 3200., // kv
        motor_r: 0.13,   // resistence
        motor_io: 0.23,  // idle current
        prop_thrust_factor: Vector3::new(-5e-05, -0.0025, 4.75),
        prop_torque_factor: 0.0056,
        prop_a_factor: 7.43e-10,
        prop_inertia: 3.5e-07,
    };

    let drone_model = DroneModel {
        frame_drag_area: Vector3::new(0.0082, 0.0077, 0.0082),
        frame_drag_constant: 1.45,
        mass: 0.2972,
        inv_tensor: Matrix3::from_diagonal(&Vector3::new(750., 5150.0, 750.0)),
    };

    let gyro_model = GyroModel {
        low_pass_filter: [
            LowPassFilter::default(),
            LowPassFilter::default(),
            LowPassFilter::default(),
        ],
    };

    let new_drone = SimulationTwo {
        current_frame: initial_frame.clone(),
        next_frame: initial_frame.clone(),
        dt: Duration::from_nanos(5000).as_secs_f64(), // kinda retarded
        battery_model,
        rotor_model,
        drone_model,
        gyro_model,
    };

    let new_sim = Simulation {
        drone: new_drone,
        flight_controller: flight_controller.clone(),
        time_accu: Duration::default(),
        dt: Duration::from_nanos(5000), // TODO: update this
    };

    commands.insert_resource(new_sim);

    // Insert the simulation debug info
    commands.insert_resource(DebugUiContent::default());

    // Insert the player controller input
    commands.insert_resource(PlayerControllerInput::default());

    // Insert the scene bundle
    commands.spawn(SceneBundle {
        scene: gltf.scenes[0].clone(),
        ..Default::default()
    });

    // Set next state
    next_state.set(SimState::Running);
}

/// The simulation loop.
fn sim_loop(
    mut gizmos: Gizmos,
    timer: Res<Time>,
    mut ui_info: ResMut<DebugUiContent>,
    mut simulation: ResMut<Simulation>,
    controller_input: Res<PlayerControllerInput>,
    mut camera_query: Query<&mut PanOrbitCamera>,
    mut scene_query: Query<(&mut Transform, &Handle<Scene>)>,
) {
    let (mut tranform, _) = scene_query.single_mut();
    let mut camera = camera_query.single_mut();
    let debug_info = simulation.simulate_delta(timer.delta(), controller_input.to_channels());

    let drone_translation = ntb_vec3(debug_info.position);
    let drone_rotation = Quat::from_mat3(&ntb_mat3(debug_info.rotation));
    tranform.translation = drone_translation;
    tranform.rotation = drone_rotation;

    let down_dir = debug_info.rotation * Vector3::new(0., -1., -0.);
    let current_frame = &simulation.drone.current_frame;
    for motor_index in 0..4 {
        let motor_pos = drone_translation
            + ntb_vec3(debug_info.rotation * current_frame.rotors_state.0[motor_index].motor_pos);
        gizmos.arrow(
            motor_pos,
            motor_pos
                + ntb_vec3(down_dir * current_frame.rotors_state.0[motor_index].effective_thrust),
            RED,
        );
    }

    *ui_info = DebugUiContent(debug_info);
    camera.target_focus = drone_translation;
}

/// The debug ui.
fn update_debug_ui(mut ctx: EguiContexts, ui_sim_info: Res<DebugUiContent>) {
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

fn main() {
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
    App::new()
        .add_plugins(default_plugin)
        .add_plugins(EguiPlugin)
        .add_plugins(PanOrbitCameraPlugin)
        .insert_state(SimState::Loading)
        .add_plugins(InfiniteGridPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, setup_drone.run_if(in_state(SimState::Loading)))
        .add_systems(Update, sim_loop.run_if(in_state(SimState::Running)))
        .add_systems(
            Update,
            store_controller_input.run_if(in_state(SimState::Running)),
        )
        .add_systems(Update, update_debug_ui.run_if(in_state(SimState::Running)))
        .run();
}
