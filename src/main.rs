mod constants;
mod drone;
mod quboid;

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use bevy_panorbit_camera::PanOrbitCameraPlugin;
use noise::{NoiseFn, Perlin};
use quboid::{cuboid_setup, handle_keyboard_events, quboid_update, update_ui};
use rand::{rngs::ThreadRng, thread_rng, Rng};
use std::{cell::RefCell, ops::Range};

thread_local! {
    static RNG: RefCell<ThreadRng> = RefCell::new(thread_rng());
    static PERLIN_NOISE: Perlin = Perlin::new(0);
}

pub fn rng_gen_range(range: Range<f64>) -> f64 {
    RNG.with(|rng| rng.borrow_mut().gen_range(range))
}

pub fn perlin_noise(point: f64) -> f64 {
    PERLIN_NOISE.with(|p| p.get([point]))
}

#[cfg(feature = "legacy_sim")]
fn build_app() -> App {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin)
        .add_plugins(PanOrbitCameraPlugin)
        .init_gizmo_group::<MyRoundGizmos>()
        .add_systems(Startup, cuboid_setup)
        .add_systems(Update, handle_keyboard_events)
        .add_systems(Update, quboid_update)
        .add_systems(Update, update_ui);
    app
}

#[derive(States, Clone, Copy, Default, Eq, PartialEq, Hash, Debug)]
enum SimState {
    #[default]
    Loading,
    Running,
}

#[cfg(not(feature = "legacy_sim"))]
fn build_app() -> App {
    use bevy_infinite_grid::InfiniteGridPlugin;
    use drone::{base_setup, setup_drone};

    let mut app = App::new();
    app.add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin)
        .add_plugins(PanOrbitCameraPlugin)
        .insert_state(SimState::Loading)
        .init_gizmo_group::<MyRoundGizmos>()
        .add_plugins(InfiniteGridPlugin)
        .add_systems(Startup, base_setup)
        .add_systems(Update, setup_drone.run_if(in_state(SimState::Loading)))
        .add_systems(
            Update,
            handle_keyboard_events.run_if(in_state(SimState::Running)),
        );
    app
}

// We can create our own gizmo config group!
#[derive(Default, Reflect, GizmoConfigGroup)]
struct MyRoundGizmos {}

fn main() {
    let mut app = build_app();
    app.run();
}

#[cfg(test)]
mod test {
    #[test]
    fn integrate_thing() {}
}
