//! The `simulation` crate provides a small bevy application that can be used to test how different
//! contfigurations may affect the drone and it's behaviour. This software is super early in
//! development, expect a lot of changes, including to how configurations are stored, what drone
//! meshes we want to use etc.
mod replay;
mod sim;
mod ui;

use crate::ui::menu::UIState;
use bevy::{
    app::{App, PluginGroup, Startup, Update},
    asset::{AssetServer, Assets, Handle},
    core_pipeline::core_3d::Camera3d,
    ecs::schedule::IntoScheduleConfigs,
    gltf::Gltf,
    math::{Mat3, Vec3},
    pbr::DirectionalLight,
    prelude::{
        default, in_state, AppExtStates, Commands, Deref, DerefMut, NextState, OnEnter, OnExit,
        Res, ResMut, Resource, States, Transform,
    },
    scene::SceneRoot,
    window::{PresentMode, Window, WindowPlugin, WindowTheme},
    DefaultPlugins,
};
use bevy_egui::{EguiGlobalSettings, EguiPlugin};
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use core::f64;
use nalgebra::{Rotation3, Vector3};
use replay::{enter_replay, exit_replay, replay_loop};
use sim::{enter_simulation, exit_simulation, handle_input, sim_loop};
use sim_context::SimContext;
use ui::draw_ui;

// Controll the visualizer state. It controls which systems are going to run.
#[derive(States, Clone, Eq, PartialEq, Hash, Debug)]
pub enum VisualizerState {
    Loading,
    Menu,
    Simulation,
    Replay,
}

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

#[derive(Resource, Deref, DerefMut, Debug)]
pub struct Context(SimContext);

/// Set up the camera, light sources, the infinite grid, and start loading the drone scene. Loading
/// glb objects in bevy is currently asyncronous and only when the scene is loaded should we
/// initialize the flight controller and start the simulation
pub fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut egui_global_settings: ResMut<EguiGlobalSettings>,
) {
    egui_global_settings.enable_absorb_bevy_input_system = true;
    commands.spawn((DirectionalLight::default(), Transform::default()));

    commands.spawn((
        Camera3d::default(),
        Transform {
            translation: Vec3::new(0.0, 1.5, 5.0),
            ..Transform::IDENTITY
        },
        PanOrbitCamera {
            ..Default::default()
        },
    ));

    commands.spawn(InfiniteGridBundle::default());
    let drone_scene = asset_server.load("drone5.glb");
    let drone_asset = DroneAsset(drone_scene);
    commands.insert_resource(drone_asset);
    commands.insert_resource(UIState::default());
    let mut context = SimContext::default();
    context.refresh_cache();
    commands.insert_resource(Context(context));
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
    commands.spawn((SceneRoot(gltf.scenes[0].clone()), Transform::default()));

    next_state.set(VisualizerState::Menu);
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

    let egui_plugin = EguiPlugin {
        enable_multipass_for_primary_context: false,
    };

    App::new()
        .add_plugins(default_plugin)
        .add_plugins(egui_plugin)
        .add_plugins(PanOrbitCameraPlugin)
        .insert_state(VisualizerState::Loading)
        .add_plugins(InfiniteGridPlugin)
        .add_systems(Startup, setup)
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
