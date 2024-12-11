//! The `simulation` crate provides a small bevy application that can be used to test how different
//! contfigurations may affect the drone and it's behaviour. This software is super early in
//! development, expect a lot of changes, including to how configurations are stored, what drone
//! meshes we want to use etc.
mod replay;
mod sim;
mod ui;

use bevy::{
    app::{App, PluginGroup, PreUpdate, Startup, Update},
    asset::{AssetServer, Assets, Handle},
    color::Color,
    gltf::Gltf,
    input::{mouse::MouseWheel, ButtonInput},
    math::{DVec3, EulerRot, Mat3, Quat, Vec3},
    pbr::{DirectionalLight, DirectionalLightBundle},
    prelude::{
        default, in_state, AppExtStates, Camera3dBundle, Commands, Deref, DerefMut, Events,
        IntoSystemConfigs, KeyCode, MouseButton, NextState, OnEnter, OnExit, Res, ResMut, Resource,
        States, Transform,
    },
    scene::SceneBundle,
    window::{PresentMode, Window, WindowPlugin, WindowTheme},
    DefaultPlugins,
};
use bevy_egui::{EguiContexts, EguiPlugin};
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use core::f64;
use db::AscentDb;
use flight_controller::controllers::bf_controller::BFController;
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};
use replay::{enter_replay, exit_replay, replay_loop, Replay};
use sim::{enter_simulation, exit_simulation, handle_input, sim_loop, PlayerControllerInput};
#[cfg(feature = "noise")]
use simulator::FrameCharachteristics;
use simulator::{
    logger::SimLogger, low_pass_filter::LowPassFilter, BatteryModel, BatteryState, Drone,
    DroneFrameState, DroneModel, GyroModel, GyroState, MotorInput, Replayer, RotorModel,
    RotorState, RotorsState, SampleCurve, SamplePoint, SimulationFrame, Simulator,
};
use std::{sync::Arc, time::Duration};
use ui::{draw_ui, SimData};

#[derive(States, Clone, Default, Eq, PartialEq, Hash, Debug)]
pub enum VisualizerState {
    #[default]
    Loading,
    Menu,
    Simulation,
    Replay,
}

// TODO: we should not rely on the mesh names for the simulation but the other way around, it would
// be nice to load and adjust the correct mesh to the simulations configuration
pub const PROP_BLADE_MESH_NAMES: [(&str, f64, DVec3); 4] = [
    (
        "prop_blade.001",
        -1.,
        DVec3::new(
            0.14055216312408447,
            0.013523973524570465,
            0.11647607386112213,
        ),
    ),
    (
        "prop_blade.002",
        1.,
        DVec3::new(
            0.14055214822292328,
            0.013523973524570465,
            -0.11647609621286392,
        ),
    ),
    (
        "prop_blade.003",
        1.,
        DVec3::new(
            -0.14055216312408447,
            0.013523973524570465,
            0.11647608131170273,
        ),
    ),
    (
        "prop_blade.004",
        -1.,
        DVec3::new(
            -0.14055216312408447,
            0.013523973524570465,
            -0.11647607386112213,
        ),
    ),
];

/// A helper function to transform an nalgebra::Vector3 to a Vec3 used by bevy
pub fn ntb_vec3(vec: Vector3<f64>) -> Vec3 {
    Vec3::new(vec[0] as f32, vec[1] as f32, vec[2] as f32)
}

/// A helper function to transform an nalgebra::Rotation3 to a Mat3 used by bevy
pub fn ntb_mat3(matrix: Rotation3<f64>) -> Mat3 {
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

/// The drone asset wrapper. It is used to query for the drone asset within the gltf assets.
#[derive(Resource, Clone, Deref)]
pub struct DroneAsset(Handle<Gltf>);

#[derive(Resource, Deref)]
struct DB(AscentDb);

// Since we currently only support a single simulation, we should use a resource for the drone and
// all the auxulary information. In the future, if we include a multi drone setup/collisions and
// other things, it might make sense to have entities/components
#[derive(Resource, Deref, DerefMut)]
pub struct Simulaton(Simulator);

pub fn initial_simulation_frame() -> SimulationFrame {
    let rotors_state =
        RotorsState(
            PROP_BLADE_MESH_NAMES.map(|(name, rotor_dir, position)| RotorState {
                current: 0.,
                rpm: 0.,
                motor_torque: 0.,
                effective_thrust: 0.,
                pwm: 0.,
                rotor_dir,
                motor_pos: Vector3::new(position.x, position.y, position.z),
                pwm_low_pass_filter: LowPassFilter::default(),
            }),
        );

    let battery_state = BatteryState {
        capacity: 850.,
        bat_voltage: 4.2,
        bat_voltage_sag: 4.2,
        amperage: 0.,
        m_ah_drawn: 0.,
    };

    let drone_state = DroneFrameState {
        position: Vector3::zeros(),
        rotation: Rotation3::identity(), // stargin position
        linear_velocity: Vector3::zeros(),
        angular_velocity: Vector3::new(0., 0., 0.),
        acceleration: Vector3::zeros(),
    };

    let gyro_state = GyroState {
        rotation: UnitQuaternion::identity(),
        acceleration: Vector3::zeros(),
        angular_velocity: Vector3::zeros(),
        low_pass_filters: [
            LowPassFilter::default(),
            LowPassFilter::default(),
            LowPassFilter::default(),
        ],
    };

    SimulationFrame {
        battery_state,
        rotors_state,
        drone_state,
        gyro_state,
    }
}

/// Set up the camera, light sources, the infinite grid, and start loading the drone scene. Loading
/// glb objects in bevy is currently asyncronous and only when the scene is loaded should we
/// initialize the flight controller and start the simulation
pub fn setup(mut commands: Commands, asset_server: Res<AssetServer>) {
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

    let db = AscentDb::new();
    let simulation_ids = db.get_all_simulation_ids();
    commands.insert_resource(SimData {
        simulation_ids,
        ..Default::default()
    });

    let flight_controller = Arc::new(BFController::new());
    let initial_frame = initial_simulation_frame();

    // TODO: we should have this entirly read from a file
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

    let gyro_model = GyroModel {};

    let drone = Drone {
        current_frame: initial_frame.clone(),
        next_frame: initial_frame.clone(),
        battery_model,
        rotor_model,
        drone_model,
        gyro_model,
    };

    let logger = SimLogger::new(MotorInput::default());

    let simulation = Simulaton(Simulator {
        drone: drone.clone(),
        flight_controller: flight_controller.clone(),
        time_accu: Duration::default(),
        time: Duration::new(0, 0),
        dt: Duration::from_nanos(5000), // TODO: update this
        fc_time_accu: Duration::default(),
        logger,
    });

    commands.insert_resource(simulation);

    let replay = Replay(Replayer {
        drone,
        time: Duration::new(0, 0),
        time_accu: Duration::new(0, 0),
        time_steps: vec![],
        replay_index: 0,
        dt: Duration::from_nanos(5000), // TODO: update this
    });

    commands.insert_resource(replay);

    // Insert the player controller input
    commands.insert_resource(PlayerControllerInput::default());

    commands.insert_resource(DB(db));
}

fn load_drone_scene(
    mut commands: Commands,
    drone_asset: Res<DroneAsset>,
    gltf_assets: Res<Assets<Gltf>>,
    // simulation: ResMut<Simulaton>,
    mut next_state: ResMut<NextState<VisualizerState>>,
) {
    let Some(gltf) = gltf_assets.get(&drone_asset.0) else {
        return;
    };

    // Insert the scene bundle
    commands.spawn(SceneBundle {
        scene: gltf.scenes[0].clone(),
        ..Default::default()
    });

    next_state.set(VisualizerState::Menu);
}

fn absorb_egui_inputs(
    mut contexts: EguiContexts,
    mut mouse: ResMut<ButtonInput<MouseButton>>,
    mut mouse_wheel: ResMut<Events<MouseWheel>>,
    mut keyboard: ResMut<ButtonInput<KeyCode>>,
) {
    let ctx = contexts.ctx_mut();
    if !(ctx.wants_pointer_input() || ctx.is_pointer_over_area()) {
        return;
    }
    let modifiers = [
        KeyCode::SuperLeft,
        KeyCode::SuperRight,
        KeyCode::ControlLeft,
        KeyCode::ControlRight,
        KeyCode::AltLeft,
        KeyCode::AltRight,
        KeyCode::ShiftLeft,
        KeyCode::ShiftRight,
    ];

    let pressed = modifiers.map(|key| keyboard.pressed(key).then_some(key));

    mouse.reset_all();
    mouse_wheel.clear();
    keyboard.reset_all();

    for key in pressed.into_iter().flatten() {
        keyboard.press(key);
    }
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
        .insert_state(VisualizerState::default())
        .add_plugins(InfiniteGridPlugin)
        .add_systems(Startup, setup)
        .add_systems(
            PreUpdate,
            absorb_egui_inputs
                .after(bevy_egui::systems::process_input_system)
                .before(bevy_egui::EguiSet::BeginPass),
        )
        .add_systems(Update, draw_ui)
        .add_systems(
            Update,
            load_drone_scene.run_if(in_state(VisualizerState::Loading)),
        )
        .add_systems(OnEnter(VisualizerState::Simulation), enter_simulation)
        .add_systems(
            Update,
            sim_loop.run_if(in_state(VisualizerState::Simulation)),
        )
        .add_systems(
            Update,
            handle_input.run_if(in_state(VisualizerState::Simulation)),
        )
        .add_systems(OnExit(VisualizerState::Simulation), exit_simulation)
        .add_systems(OnEnter(VisualizerState::Replay), enter_replay)
        .add_systems(
            Update,
            replay_loop.run_if(in_state(VisualizerState::Replay)),
        )
        .add_systems(OnExit(VisualizerState::Replay), exit_replay)
        .run();
}
