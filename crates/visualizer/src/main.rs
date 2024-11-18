mod controller;
mod game_loop;
mod setup;
mod ui;

use bevy::{
    app::{App, PluginGroup, Startup, Update},
    gizmos::AppGizmoBuilder,
    prelude::{
        default, in_state, AppExtStates, Component, Deref, DerefMut, GizmoConfigGroup,
        IntoSystemConfigs, States,
    },
    reflect::Reflect,
    window::{PresentMode, Window, WindowPlugin, WindowTheme},
    DefaultPlugins,
};
use bevy_egui::EguiPlugin;
use bevy_infinite_grid::InfiniteGridPlugin;
use bevy_panorbit_camera::PanOrbitCameraPlugin;
use controller::gamepad_input_events;
use core::f64;
use flight_controller::{Channels, FlightController};
use game_loop::debug_drone;
use setup::{base_setup, setup_drone};
use simulator::Drone;
#[cfg(feature = "noise")]
use simulator::FrameCharachteristics;
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

#[derive(Default, Reflect, GizmoConfigGroup)]
struct MyRoundGizmos {}

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
        // .add_systems(Update, gamepad_system.run_if(in_state(SimState::Running)))
        .add_systems(Update, update_ui.run_if(in_state(SimState::Running)));
    app
}

fn main() {
    let mut app = build_app();
    app.run();
}
