mod controller;
mod game_loop;
mod setup;
mod ui;

use bevy::{
    app::{App, PluginGroup, Startup, Update},
    gizmos::AppGizmoBuilder,
    prelude::{
        default, in_state, AppExtStates, Component, GizmoConfigGroup, IntoSystemConfigs, Query,
        States,
    },
    reflect::Reflect,
    window::{PresentMode, Window, WindowPlugin, WindowTheme},
    DefaultPlugins,
};
use bevy_egui::EguiPlugin;
use bevy_infinite_grid::InfiniteGridPlugin;
use bevy_panorbit_camera::PanOrbitCameraPlugin;
use core::f64;
use flight_controller::{
    controllers::bf_controller::BFController, BatteryUpdate, Channels, FlightController,
    FlightControllerUpdate, GyroUpdate, MotorInput,
};
use game_loop::debug_drone;
use quad_sim::Drone;
#[cfg(feature = "noise")]
use quad_sim::FrameCharachteristics;
use setup::{base_setup, setup_drone};
use std::{sync::Arc, time::Duration};
use ui::update_ui;

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
    fn new() -> Self {
        Self {
            fc: Arc::new(BFController::new()),
        }
    }

    fn init(&self) {
        self.fc.init()
    }

    fn update(&self, update: FlightControllerUpdate) -> Option<MotorInput> {
        self.fc.update(update)
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
            dt: Duration::from_nanos(50000), // TODO: update this
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

#[derive(Component, Default)]
struct Controller(Channels);

impl Controller {
    pub fn channels(&self) -> Channels {
        self.0
    }
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
