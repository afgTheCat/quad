//! The `simulation` crate provides a small bevy application that can be used to test how different
//! contfigurations may affect the drone and it's behaviour. This software is super early in
//! development, expect a lot of changes, including to how configurations are stored, what drone
//! meshes we want to use etc.
// mod drone;
mod replay;
mod sim;
mod ui;

use bevy::{
    app::{App, PluginGroup, PreUpdate, Startup, Update},
    asset::{AssetServer, Assets, Handle},
    color::Color,
    gltf::Gltf,
    input::{mouse::MouseWheel, ButtonInput},
    math::{EulerRot, Mat3, Quat, Vec3},
    pbr::{DirectionalLight, DirectionalLightBundle},
    prelude::{
        default, in_state, AppExtStates, Camera3dBundle, Commands, Deref, Events,
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
use nalgebra::{Rotation3, Vector3};
use replay::{enter_replay, exit_replay, replay_loop};
use sim::{enter_simulation, exit_simulation, handle_input, sim_loop};
use simulator::loader::SimLoader;
use std::sync::Arc;
use ui::{draw_ui, menu::SelectionConfig, prefetch_menu_items};

// Controll the visualizer state. It controls which systems are going to run.
#[derive(States, Clone, Eq, PartialEq, Hash, Debug)]
pub enum VisualizerState {
    Loading,
    Menu,
    Simulation,
    Replay,
}

// Global data for everything related to the visualizer
#[derive(Resource, Default)]
pub struct VisualizerData {
    pub simulation_ids: Vec<String>,
    pub reservoir_ids: Vec<String>,
    pub selection_config: SelectionConfig,
}

// replay info
// TODO: figure this out!
#[derive(Resource, Default)]
pub struct ReplayInfo {}

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
struct DB(Arc<AscentDb>);

#[derive(Resource, Deref)]
struct Loader(SimLoader);

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
    commands.insert_resource(drone_asset);
    commands.insert_resource(VisualizerData {
        ..Default::default()
    });

    let db = Arc::new(AscentDb::new("/home/gabor/ascent/quad/data.sqlite"));
    commands.insert_resource(DB(db.clone()));
    let sim_loader = SimLoader::new(db);
    commands.insert_resource(Loader(sim_loader));
}

fn load_drone_scene(
    mut commands: Commands,
    drone_asset: Res<DroneAsset>,
    gltf_assets: Res<Assets<Gltf>>,
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
        .insert_state(VisualizerState::Loading)
        .add_plugins(InfiniteGridPlugin)
        .add_systems(Startup, setup)
        .add_systems(
            PreUpdate,
            absorb_egui_inputs
                .after(bevy_egui::systems::process_input_system)
                .before(bevy_egui::EguiSet::BeginPass),
        )
        .add_systems(Update, draw_ui)
        .add_systems(OnEnter(VisualizerState::Menu), prefetch_menu_items)
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
