// #[cfg(feature = "legacy_sim")]
// use quboid::build_app;

#[cfg(feature = "legacy_sim")]
use quboid_bevy::build_app;

#[cfg(not(feature = "legacy_sim"))]
use quad_bevy::build_app;

fn main() {
    let mut app = build_app();
    app.run();
}
