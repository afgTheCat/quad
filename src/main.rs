mod constants;
mod low_pass_filter;
mod sample_curve;

use noise::{NoiseFn, Perlin};
use rand::{rngs::ThreadRng, thread_rng, Rng};
use std::{cell::RefCell, ops::Range};

#[cfg(feature = "legacy_sim")]
use quboid::build_app;

#[cfg(feature = "legacy_sim")]
mod quboid;

#[cfg(not(feature = "legacy_sim"))]
mod sim;

#[cfg(not(feature = "legacy_sim"))]
use sim::build_app;

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

// We can create our own gizmo config group!

fn main() {
    let mut app = build_app();
    app.run();
}

#[cfg(test)]
mod test {
    #[test]
    fn integrate_thing() {}
}
