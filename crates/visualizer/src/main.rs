//! The `simulation` crate provides a small bevy application that can be used to test how different
//! contfigurations may affect the drone and it's behaviour. This software is super early in
//! development, expect a lot of changes, including to how configurations are stored, what drone
//! meshes we want to use etc.
mod replay;
mod sim;

use bevy::{
    app::{App, PluginGroup, Startup, Update},
    asset::{AssetServer, Handle},
    color::Color,
    gltf::Gltf,
    math::{EulerRot, Mat3, Quat, Vec3},
    pbr::{DirectionalLight, DirectionalLightBundle},
    prelude::{
        default, in_state, AppExtStates, Camera3dBundle, Commands, Deref, IntoSystemConfigs,
        NextState, Res, ResMut, Resource, State, States, Transform,
    },
    window::{PresentMode, Window, WindowPlugin, WindowTheme},
    DefaultPlugins,
};
use bevy_egui::{egui::Window as EguiWindow, EguiContexts, EguiPlugin};
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use core::f64;
use nalgebra::{Rotation3, Vector3, Vector4};
use sim::{handle_input, setup_drone, sim_loop, update_debug_ui};
#[cfg(feature = "noise")]
use simulator::FrameCharachteristics;
use std::sync::Arc;

#[derive(Debug, Default)]
pub struct SimulationDebugInfo {
    pub rotation: Rotation3<f64>,
    pub position: Vector3<f64>,
    pub linear_velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    pub thrusts: Vector4<f64>,
    pub rpms: Vector4<f64>,
    pub pwms: Vector4<f64>,
    pub bat_voltage: f64,
    pub bat_voltage_sag: f64,
}

#[derive(States, Clone, Copy, Default, Eq, PartialEq, Hash, Debug)]
pub enum UiState {
    #[default]
    Menu,
    Simulation,
    Replay,
}

#[derive(Debug, Clone)]
pub enum UiData {
    SimulationInfo(Arc<SimulationDebugInfo>),
    ReplayInfo,
}

impl UiData {
    fn as_simulation_debug_info(&self) -> Arc<SimulationDebugInfo> {
        let UiData::SimulationInfo(data) = self.clone() else {
            panic!("")
        };
        data
    }
}

// TODO: we should not rely on the mesh names for the simulation
pub const PROP_BLADE_MESH_NAMES: [(&str, f64); 4] = [
    ("prop_blade.001", -1.),
    ("prop_blade.002", 1.),
    ("prop_blade.003", 1.),
    ("prop_blade.004", -1.),
];

/// The simulation has two states: when the drone mesh is loading, and when the simulation is
/// running. In the future we would like to implement a stopped state as well in order to enable
/// simulation resetting
#[derive(States, Clone, Copy, Default, Eq, PartialEq, Hash, Debug)]
pub enum VisualizerState {
    #[default]
    Unselected, // currently selecting which state should the visualizer be in
    LiveLoading,
    LiveRunning,
    ReplayLoading,
    ReplayRunning,
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
}

pub fn mode_selector(
    mut ctx: EguiContexts,
    mut next_visualizer_state: ResMut<NextState<VisualizerState>>,
) {
    EguiWindow::new("Mode selector").show(ctx.ctx_mut(), |ui| {
        if ui.button("Live").clicked() {
            next_visualizer_state.set(VisualizerState::LiveLoading);
        }
        if ui.button("Replayer").clicked() {
            next_visualizer_state.set(VisualizerState::ReplayLoading);
        }
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
        .insert_state(VisualizerState::default())
        .insert_state(UiState::default())
        .add_plugins(InfiniteGridPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, mode_selector.run_if(in_state(UiState::Menu)))
        .add_systems(
            Update,
            setup_drone.run_if(in_state(VisualizerState::LiveLoading)),
        )
        .add_systems(
            Update,
            sim_loop.run_if(in_state(VisualizerState::LiveRunning)),
        )
        .add_systems(
            Update,
            handle_input.run_if(in_state(VisualizerState::LiveRunning)),
        )
        .add_systems(
            Update,
            update_debug_ui.run_if(in_state(VisualizerState::LiveRunning)),
        )
        .run();
}
